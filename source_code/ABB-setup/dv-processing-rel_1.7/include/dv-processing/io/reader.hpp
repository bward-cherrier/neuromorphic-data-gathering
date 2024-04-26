#pragma once

#include "compression/decompression_support.hpp"
#include "stream.hpp"

#include <boost/endian.hpp>

#include <optional>
#include <unordered_map>
#include <utility>

namespace dv::io {

class Reader {
public:
	using ReadHandler = dv::std_function_exact<void(std::vector<std::byte> &, const int64_t)>;

	explicit Reader(dv::io::support::TypeResolver resolver   = dv::io::support::defaultTypeResolver,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr) :
		mTypeResolver(std::move(resolver)),
		mStats(std::move(stats)) {
	}

	~Reader() = default;

	Reader(const Reader &other)                = delete;
	Reader &operator=(const Reader &other)     = delete;
	Reader(Reader &&other) noexcept            = default;
	Reader &operator=(Reader &&other) noexcept = default;

	void verifyVersion(const ReadHandler &readHandler) {
		readFromInput(dv::EnumAsInteger(dv::Constants::AEDAT_VERSION_LENGTH), 0, readHandler);

		// Check if file is really AEDAT 4.0.
		if (std::memcmp(mReadBuffer.data(), dv::io::support::AEDAT4_HEADER_VERSION.data(),
				dv::io::support::AEDAT4_HEADER_VERSION.length())
			!= 0) {
			throw std::runtime_error("AEDAT4.0: no valid version line found.");
		}
	}

	[[nodiscard]] std::unique_ptr<const dv::IOHeader> readHeader(const ReadHandler &readHandler) {
		// We want to understand what data is in this data stream,
		// and return actionable elements to our caller.
		// Parse version 4.0 header content.
		// Uses IOHeader flatbuffer.
		// Get its size first.
		readFromInput(sizeof(flatbuffers::uoffset_t), -1, readHandler);

		// Size is little-endian.
		const flatbuffers::uoffset_t ioHeaderSize
			= boost::endian::load_little_u32(reinterpret_cast<const unsigned char *>(mReadBuffer.data()));

		readFromInput(ioHeaderSize, -1, readHandler);

		auto header = decodeHeader(mReadBuffer);

		mDecompressionSupport = dv::io::compression::createDecompressionSupport(header->compression);

		dv::io::support::XMLConfigReader xmlConfig{header->infoNode, "outInfo"};

		for (const auto &node : xmlConfig.getRoot().mChildren) {
			// Check that original name info exists.
			const auto nameIter = std::find(node.mAttributes.cbegin(), node.mAttributes.cend(), "originalOutputName");
			if (nameIter == node.mAttributes.cend()) {
				// Original name doesn't exist, skip.
				continue;
			}

			auto originalOutputName = std::get<std::string>(nameIter->mValue);

			// Check that type info exists.
			const auto typeIter = std::find(node.mAttributes.cbegin(), node.mAttributes.cend(), "typeIdentifier");
			if (typeIter == node.mAttributes.cend()) {
				// Type identifier doesn't exist, skip.
				continue;
			}

			auto typeIdentifier = std::get<std::string>(typeIter->mValue);

			const auto *typeInfo = mTypeResolver(dv::types::IdentifierStringToId(typeIdentifier));
			if (typeInfo == nullptr) {
				// Type not available, skip.
				continue;
			}

			const int32_t streamId = std::stoi(node.mName);

			mStreams[streamId].mType           = *typeInfo;
			mStreams[streamId].mTypeIdentifier = std::move(typeIdentifier);
			mStreams[streamId].mId             = streamId;
			mStreams[streamId].mName           = std::move(originalOutputName);
			mStreams[streamId].mXMLNode        = node;
		}

		return header;
	}

	[[nodiscard]] std::unique_ptr<const dv::FileDataTable> readFileDataTable(
		const uint64_t size, const int64_t position, const ReadHandler &readHandler) {
		readFromInput(size, position, readHandler);

		decompressData();

		return decodeFileDataTable(mDecompressBuffer);
	}

	[[nodiscard]] std::tuple<dv::PacketHeader, std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes>
		readPacket(const ReadHandler &readHandler) {
		return readPacket(-1, readHandler);
	}

	[[nodiscard]] std::tuple<dv::PacketHeader, std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes>
		readPacket(const int64_t byteOffset, const ReadHandler &readHandler) {
		auto header = readPacketHeader(byteOffset, readHandler);

		const auto dataSize   = static_cast<uint64_t>(header.Size());
		const auto dataOffset = byteOffset + static_cast<int64_t>(sizeof(dv::PacketHeader));

		auto [body, sizes] = readPacketBody(header.StreamID(), dataSize, dataOffset, readHandler);

		return std::make_tuple(header, std::move(body), sizes);
	}

	[[nodiscard]] dv::PacketHeader readPacketHeader(const ReadHandler &readHandler) {
		return readPacketHeader(-1, readHandler);
	}

	[[nodiscard]] dv::PacketHeader readPacketHeader(const int64_t byteOffset, const ReadHandler &readHandler) {
		readFromInput(sizeof(dv::PacketHeader), byteOffset, readHandler);

		return *reinterpret_cast<dv::PacketHeader *>(mReadBuffer.data());
	}

	[[nodiscard]] std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes> readPacketBody(
		const dv::FileDataDefinition &packet, const ReadHandler &readHandler) {
		return readPacketBody(packet.PacketInfo.StreamID(), static_cast<uint64_t>(packet.PacketInfo.Size()),
			packet.ByteOffset, readHandler);
	}

	[[nodiscard]] std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes> readPacketBody(
		const int32_t streamId, const uint64_t size, const ReadHandler &readHandler) {
		return readPacketBody(streamId, size, -1, readHandler);
	}

	[[nodiscard]] std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes> readPacketBody(
		const int32_t streamId, const uint64_t size, const int64_t byteOffset, const ReadHandler &readHandler) {
		dv::io::support::Sizes sizes;

		readFromInput(size, byteOffset, readHandler);
		sizes.mDataSize = mReadBuffer.size();

		decompressData();
		sizes.mPacketSize = mDecompressBuffer.size();

		auto decodedPacketBody = decodePacketBody(mDecompressBuffer, mStreams[streamId].mType);

		dv::types::TimeElementExtractor extractedInfo{};
		decodedPacketBody->type.timeElementExtractor(decodedPacketBody->obj, &extractedInfo);

		sizes.mPacketElements = static_cast<uint64_t>(extractedInfo.numElements);

		if (mStats) {
			// AddedDataSize is zero because readInput() already takes care of it.
			mStats->update(0, 1, sizes.mPacketElements, sizes.mPacketSize);
		}

		return std::make_pair(std::move(decodedPacketBody), sizes);
	}

	[[nodiscard]] std::unique_ptr<const dv::FileDataTable> buildFileDataTable(
		const uint64_t fileSize, const ReadHandler &readHandler) {
		auto dataTable = std::make_unique<dv::FileDataTable>();
		dataTable->Table.reserve(2048); // cvector starts empty.

		// Read the size of the header, which comes right after the aedat version string
		readFromInput(
			sizeof(flatbuffers::uoffset_t), dv::EnumAsInteger(dv::Constants::AEDAT_VERSION_LENGTH), readHandler);

		// Size is little-endian.
		const flatbuffers::uoffset_t ioHeaderSize
			= boost::endian::load_little_u32(reinterpret_cast<const unsigned char *>(mReadBuffer.data()));

		const auto firstPacketPosition
			= dv::EnumAsInteger(dv::Constants::AEDAT_VERSION_LENGTH) + sizeof(flatbuffers::uoffset_t) + ioHeaderSize;

		auto nextRead = firstPacketPosition;
		while ((nextRead + sizeof(dv::PacketHeader)) < fileSize) {
			const auto header = readPacketHeader(static_cast<int64_t>(nextRead), readHandler);
			nextRead          += sizeof(dv::PacketHeader);

			const auto dataOffset = nextRead;
			const auto dataSize   = static_cast<uint64_t>(header.Size());

			if ((nextRead + dataSize) > fileSize) {
				break;
			}

			const auto [body, sizes]
				= readPacketBody(header.StreamID(), dataSize, static_cast<int64_t>(nextRead), readHandler);
			nextRead += dataSize;

			dv::types::TimeElementExtractor extractedInfo{};
			body->type.timeElementExtractor(body->obj, &extractedInfo);

			// Only add to table if reading and parsing was fully possible.
			dataTable->Table.emplace_back(static_cast<int64_t>(dataOffset), header, extractedInfo.numElements,
				extractedInfo.startTimestamp, extractedInfo.endTimestamp);
		}

		return dataTable;
	}

	[[nodiscard]] std::vector<dv::io::Stream> getStreams() const {
		std::vector<dv::io::Stream> streams;

		for (const auto &[key, val] : mStreams) {
			streams.push_back(val);
		}

		std::sort(streams.begin(), streams.end(), [](const auto &a, const auto &b) {
			return (a.mId < b.mId);
		});

		streams.shrink_to_fit();

		return streams;
	}

	[[nodiscard]] CompressionType getCompressionType() const {
		return mDecompressionSupport->getCompressionType();
	}

private:
	dv::io::support::TypeResolver mTypeResolver;
	std::unique_ptr<dv::io::support::IOStatistics> mStats;
	std::unique_ptr<dv::io::compression::DecompressionSupport> mDecompressionSupport;
	std::vector<std::byte> mReadBuffer;
	std::vector<std::byte> mDecompressBuffer;
	std::unordered_map<int32_t, dv::io::Stream> mStreams;

	[[nodiscard]] static std::unique_ptr<const dv::IOHeader> decodeHeader(const std::vector<std::byte> &header) {
		// Verify header content is valid.
		flatbuffers::Verifier ioHeaderVerify(reinterpret_cast<const uint8_t *>(header.data()), header.size());

		if (!dv::VerifyIOHeaderBuffer(ioHeaderVerify)) {
			throw std::runtime_error("AEDAT4.0: could not verify IOHeader contents.");
		}

		return dv::UnPackIOHeader(header.data());
	}

	[[nodiscard]] static std::unique_ptr<const dv::FileDataTable> decodeFileDataTable(
		const std::vector<std::byte> &table) {
		// Verify data table is valid.
		flatbuffers::Verifier dataTableVerify(
			reinterpret_cast<const uint8_t *>(table.data()), table.size(), 64, INT32_MAX);

		if (!dv::VerifySizePrefixedFileDataTableBuffer(dataTableVerify)) {
			throw std::runtime_error("AEDAT4.0: could not verify FileDataTable contents.");
		}

		// Unpack table.
		const auto dataTableFlatbuffer = dv::GetSizePrefixedFileDataTable(table.data());

		return std::unique_ptr<const dv::FileDataTable>(dataTableFlatbuffer->UnPack());
	}

	[[nodiscard]] static std::unique_ptr<dv::types::TypedObject> decodePacketBody(
		const std::vector<std::byte> &packet, const dv::types::Type &type) {
		const auto typeIdentifier = dv::types::IdToIdentifierString(type.id);
		if (!flatbuffers::BufferHasIdentifier(packet.data(), typeIdentifier.data(), true)) {
			// Wrong type identifier for this flatbuffer (file_identifier field).
			// This should never happen, ignore packet.
			throw std::runtime_error(fmt::format("Wrong type identifier for packet: '{:s}', expected: '{:s}'.",
				flatbuffers::GetBufferIdentifier(packet.data(), true), typeIdentifier.data()));
		}

		const auto fbPtr = flatbuffers::GetSizePrefixedRoot<void>(packet.data());

		auto typedObj = std::make_unique<dv::types::TypedObject>(type);

		type.unpack(typedObj->obj, fbPtr);

		return typedObj;
	}

	void readFromInput(const uint64_t length, const int64_t position, const ReadHandler &readHandler) {
		mReadBuffer.resize(length);
		readHandler(mReadBuffer, position);

		if (mStats) {
			mStats->addBytes(length);
		}
	}

	void decompressData() {
		mDecompressionSupport->decompress(mReadBuffer, mDecompressBuffer);
	}
};

} // namespace dv::io

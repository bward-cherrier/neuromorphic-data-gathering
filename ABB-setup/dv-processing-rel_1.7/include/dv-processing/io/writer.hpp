#pragma once

#include "compression/compression_support.hpp"
#include "support/utils.hpp"

#include <iostream>
#include <memory>

namespace dv::io {

class Writer {
public:
	using WriteHandler = dv::std_function_exact<void(const std::shared_ptr<const dv::io::support::IODataBuffer>)>;

	Writer() = delete;

	explicit Writer(std::unique_ptr<dv::io::compression::CompressionSupport> compression,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr,
		std::unique_ptr<dv::FileDataTable> dataTable         = nullptr) :
		mStats(std::move(stats)),
		mCompressionSupport(std::move(compression)),
		mFileDataTable(std::move(dataTable)) {
	}

	explicit Writer(const dv::CompressionType compression,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr,
		std::unique_ptr<dv::FileDataTable> dataTable         = nullptr) :
		Writer(dv::io::compression::createCompressionSupport(compression), std::move(stats), std::move(dataTable)) {
	}

	~Writer() = default;

	Writer(const Writer &other)                = delete;
	Writer &operator=(const Writer &other)     = delete;
	Writer(Writer &&other) noexcept            = default;
	Writer &operator=(Writer &&other) noexcept = default;

	[[nodiscard]] auto getCompressionType() {
		return mCompressionSupport->getCompressionType();
	}

	size_t writeAedatVersion(const WriteHandler &writeHandler) {
		const auto aedat4Version = encodeAedat4Version();

		writeToDestination(aedat4Version, writeHandler);
		const auto written = aedat4Version->getDataSize();

		if (mStats) {
			mStats->addBytes(written);
		}

		mByteOffset += written;
		return written;
	}

	size_t writeHeader(
		const int64_t dataTablePosition, const std::string_view infoNode, const WriteHandler &writeHandler) {
		const auto encodedHeader = encodeFileHeader(dataTablePosition, infoNode, getCompressionType());

		writeToDestination(encodedHeader, writeHandler);
		const auto written = encodedHeader->getDataSize();

		// Ignore other writes with data table position in file,
		// as those don't change the file size at all.
		if (mStats && (dataTablePosition == -1)) {
			mStats->addBytes(written);
		}

		mByteOffset += written;
		return written;
	}

	size_t writePacket(
		const dv::types::TypedObject *const packet, const int32_t streamId, const WriteHandler &writeHandler) {
		dv::types::TimeElementExtractor timeElementInfo{};
		packet->type.timeElementExtractor(packet->obj, &timeElementInfo);

		const auto encodedPacket = encodePacketBody(packet->obj, packet->type);
		const auto packetSize    = encodedPacket->getDataSize();

		compressData(*encodedPacket);

		encodePacketHeader(encodedPacket, streamId);
		const auto written = encodedPacket->getDataSize() + sizeof(dv::PacketHeader);

		writeToDestination(encodedPacket, writeHandler);

		if (mStats) {
			mStats->update(written, 1, static_cast<uint64_t>(timeElementInfo.numElements), packetSize);
		}

		if (mFileDataTable) {
			updateFileDataTable(mByteOffset + sizeof(dv::PacketHeader),
				static_cast<uint64_t>(timeElementInfo.numElements), timeElementInfo.startTimestamp,
				timeElementInfo.endTimestamp, *encodedPacket->getHeader());
		}

		mByteOffset += written;
		return written;
	}

	size_t writePacket(
		const void *ptr, const dv::types::Type &type, const int32_t streamId, const WriteHandler &writeHandler) {
		dv::types::TimeElementExtractor timeElementInfo{};
		type.timeElementExtractor(ptr, &timeElementInfo);

		const auto encodedPacket = encodePacketBody(ptr, type);
		const auto packetSize    = encodedPacket->getDataSize();

		compressData(*encodedPacket);

		encodePacketHeader(encodedPacket, streamId);
		const auto written = encodedPacket->getDataSize() + sizeof(dv::PacketHeader);

		writeToDestination(encodedPacket, writeHandler);

		if (mStats) {
			mStats->update(written, 1, static_cast<uint64_t>(timeElementInfo.numElements), packetSize);
		}

		if (mFileDataTable) {
			updateFileDataTable(mByteOffset + sizeof(dv::PacketHeader),
				static_cast<uint64_t>(timeElementInfo.numElements), timeElementInfo.startTimestamp,
				timeElementInfo.endTimestamp, *encodedPacket->getHeader());
		}

		mByteOffset += written;
		return written;
	}

	int64_t writeFileDataTable(const WriteHandler &writeHandler) {
		auto encodedTable = encodeFileDataTable(*mFileDataTable);

		compressData(*encodedTable);
		writeToDestination(encodedTable, writeHandler);

		const auto size     = encodedTable->getDataSize();
		const auto position = mByteOffset;

		if (mStats) {
			mStats->addBytes(size);
		}

		mByteOffset += size;
		return static_cast<int64_t>(position);
	}

	[[nodiscard]] static std::shared_ptr<dv::io::support::IODataBuffer> encodeAedat4Version() {
		auto version = std::make_shared<dv::io::support::IODataBuffer>();
		version->switchToBuffer();

		version->getBuffer()->resize(dv::io::support::AEDAT4_HEADER_VERSION.length());
		std::memcpy(version->getBuffer()->data(), dv::io::support::AEDAT4_HEADER_VERSION.data(),
			dv::io::support::AEDAT4_HEADER_VERSION.length());

		return version;
	}

	[[nodiscard]] static std::shared_ptr<dv::io::support::IODataBuffer> encodeFileHeader(
		const int64_t dataTablePosition, const std::string_view infoNode, const dv::CompressionType compressionType) {
		dv::IOHeader header;

		header.compression       = compressionType;
		header.dataTablePosition = dataTablePosition;
		header.infoNode          = infoNode;

		auto encodedHeader = std::make_shared<dv::io::support::IODataBuffer>();

		// Force writing of all IOHeader Fields even if they contain default values,
		// so that the size of the written structure never changes.
		encodedHeader->getBuilder()->ForceDefaults(true);

		const auto headerOffset = dv::IOHeaderFlatbuffer::Pack(*encodedHeader->getBuilder(), &header);
		dv::FinishSizePrefixedIOHeaderBuffer(*encodedHeader->getBuilder(), headerOffset);

		return encodedHeader;
	}

	static void encodePacketHeader(
		const std::shared_ptr<dv::io::support::IODataBuffer> packet, const int32_t streamId) {
		*packet->getHeader() = PacketHeader(streamId, static_cast<int32_t>(packet->getDataSize()));
	}

	[[nodiscard]] static std::shared_ptr<dv::io::support::IODataBuffer> encodePacketBody(
		const void *ptr, const dv::types::Type &type) {
		const auto typeIdentifier = dv::types::IdToIdentifierString(type.id);

		auto encodedPacket = std::make_shared<dv::io::support::IODataBuffer>();

		const auto offset = type.pack(encodedPacket->getBuilder(), ptr);
		encodedPacket->getBuilder()->FinishSizePrefixed(flatbuffers::Offset<void>(offset), typeIdentifier.data());

		return encodedPacket;
	}

	[[nodiscard]] static std::shared_ptr<dv::io::support::IODataBuffer> encodeFileDataTable(
		const dv::FileDataTable &table) {
		auto encodedTable = std::make_shared<dv::io::support::IODataBuffer>();

		const auto offset = dv::FileDataTableFlatbuffer::Pack(*encodedTable->getBuilder(), &table);
		dv::FinishSizePrefixedFileDataTableBuffer(*encodedTable->getBuilder(), offset);

		return encodedTable;
	}

private:
	std::unique_ptr<dv::io::support::IOStatistics> mStats;
	std::unique_ptr<dv::io::compression::CompressionSupport> mCompressionSupport;
	std::unique_ptr<dv::FileDataTable> mFileDataTable;
	uint64_t mByteOffset{0};

	void writeToDestination(
		const std::shared_ptr<const dv::io::support::IODataBuffer> data, const WriteHandler &writeHandler) {
		writeHandler(data);
	}

	void compressData(dv::io::support::IODataBuffer &packet) {
		mCompressionSupport->compress(packet);
	}

	void updateFileDataTable(const uint64_t byteOffset, const uint64_t numElements, const int64_t timestampStart,
		const int64_t timestampEnd, const dv::PacketHeader &header) {
		// Update packet data table.
		auto dataDef = dv::FileDataDefinition();

		dataDef.ByteOffset     = static_cast<int64_t>(byteOffset);
		dataDef.PacketInfo     = header;
		dataDef.NumElements    = static_cast<int64_t>(numElements);
		dataDef.TimestampStart = timestampStart;
		dataDef.TimestampEnd   = timestampEnd;

		mFileDataTable->Table.push_back(dataDef);
	}
};

} // namespace dv::io

#pragma once

#include "reader.hpp"
#include "simplefile.hpp"

namespace dv::io {

struct FileInfo {
	uint64_t mFileSize;
	dv::CompressionType mCompression;
	int64_t mDataTablePosition;
	int64_t mDataTableSize;
	dv::FileDataTable mDataTable;
	int64_t mTimeLowest;
	int64_t mTimeHighest;
	int64_t mTimeDifference;
	int64_t mTimeShift;
	std::vector<dv::io::Stream> mStreams;
	std::unordered_map<int32_t, dv::FileDataTable> mPerStreamDataTables;
};

class ReadOnlyFile : private dv::io::SimpleReadOnlyFile {
public:
	ReadOnlyFile() = delete;

	explicit ReadOnlyFile(const std::filesystem::path &filePath,
		const dv::io::support::TypeResolver &resolver        = dv::io::support::defaultTypeResolver,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr) :
		dv::io::SimpleReadOnlyFile(filePath),
		mReader(resolver, std::move(stats)) {
		if ((filePath.extension().string() != dv::io::support::AEDAT4_FILE_EXTENSION)) {
			throw dv::exceptions::FileNotFound(fmt::format("File extension '{}' is not the required '{}'.",
												   filePath.extension(), dv::io::support::AEDAT4_FILE_EXTENSION),
				filePath);
		}

		parseHeader();
	}

	[[nodiscard]] const auto &getFileInfo() const {
		return mFileInfo;
	}

	/**
	 * Return all packets containing data with timestamps between a given start and end timestamp, meaning
	 * all data with a timestamp in [start, end].

	 * @param startTimestamp start timestamp of range, inclusive.
	 * @param endTimestamp end timestamp of range, inclusive.
	 * @param streamId data stream ID (separate logical type).
	 *
	 * @return packets containing data within given timestamp range.
	 */
	[[nodiscard]] std::vector<std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes>> read(
		const int64_t startTimestamp, const int64_t endTimestamp, const int32_t streamId) {
		std::vector<std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes>> data;

		auto iter = mFileInfo.mPerStreamDataTables.find(streamId);

		if ((iter == mFileInfo.mPerStreamDataTables.cend()) || (iter->second.Table.empty())) {
			return data;
		}

		for (auto packet = getStartingPointForTimeRangeSearch(startTimestamp, iter->second);
			 packet != iter->second.Table.cend(); packet++) {
			if (inRange(startTimestamp, endTimestamp, *packet)) {
				data.emplace_back(read(*packet));
			}
			else if (pastRange(startTimestamp, endTimestamp, *packet)) {
				break;
			}
		}

		return data;
	}

	[[nodiscard]] std::pair<std::unique_ptr<dv::types::TypedObject>, const dv::io::support::Sizes> read(
		const dv::FileDataDefinition &packet) {
		return mReader.readPacketBody(packet, [this](std::vector<std::byte> &data, const int64_t pos) {
			readClbk(data, pos);
		});
	}

	[[nodiscard]] std::pair<std::unique_ptr<const dv::types::TypedObject>, const dv::io::support::Sizes> read(
		const int32_t streamId, const uint64_t size, const int64_t byteOffset) {
		return mReader.readPacketBody(
			streamId, size, byteOffset, [this](std::vector<std::byte> &data, const int64_t pos) {
				readClbk(data, pos);
			});
	}

	[[nodiscard]] static bool inRange(
		const int64_t rangeStart, const int64_t rangeEnd, const dv::FileDataDefinition &packet) {
		return ((packet.TimestampStart <= rangeEnd) && (packet.TimestampEnd >= rangeStart));
	}

	[[nodiscard]] static bool aheadOfRange(
		const int64_t rangeStart, [[maybe_unused]] const int64_t rangeEnd, const dv::FileDataDefinition &packet) {
		return (packet.TimestampEnd < rangeStart);
	}

	[[nodiscard]] static bool pastRange(
		[[maybe_unused]] const int64_t rangeStart, const int64_t rangeEnd, const dv::FileDataDefinition &packet) {
		return (packet.TimestampStart > rangeEnd);
	}

private:
	dv::io::FileInfo mFileInfo;
	dv::io::Reader mReader;

	void parseHeader() {
		mReader.verifyVersion([this](std::vector<std::byte> &data, const int64_t pos) {
			readClbk(data, pos);
		});

		const auto header = mReader.readHeader([this](std::vector<std::byte> &data, const int64_t pos) {
			readClbk(data, pos);
		});

		// Set file-level return information.
		mFileInfo.mFileSize          = fileSize();
		mFileInfo.mCompression       = header->compression;
		mFileInfo.mDataTablePosition = header->dataTablePosition;
		mFileInfo.mDataTableSize     = (mFileInfo.mDataTablePosition < 0)
										 ? (0)
										 : (static_cast<int64_t>(mFileInfo.mFileSize) - mFileInfo.mDataTablePosition);

		// Get file data table to determine seek position.
		loadFileDataTable();

		createFileInfo();
	}

	void loadFileDataTable() {
		// Sanity check: data table size cannot be negative or zero IF the table should be present.
		if ((mFileInfo.mDataTablePosition >= 0) && (mFileInfo.mDataTableSize <= 0)) {
			throw std::runtime_error("FileDataTable set but not present, truncated/corrupt file.");
		}

		// Remember offset to go back here after.
		const auto initialOffset = tell();

		// Only proceed if data table is present.
		if (mFileInfo.mDataTablePosition < 0) {
			// Rebuild table if not present. Can be slow.
			mFileInfo.mDataTable = *mReader.buildFileDataTable(
				mFileInfo.mFileSize, [this](std::vector<std::byte> &data, const int64_t pos) {
					readClbk(data, pos);
				});
		}
		else {
			mFileInfo.mDataTable = *mReader.readFileDataTable(mFileInfo.mDataTableSize, mFileInfo.mDataTablePosition,
				[this](std::vector<std::byte> &data, const int64_t pos) {
					readClbk(data, pos);
				});
		}

		// Reset file position to initial one.
		seek(initialOffset);
	}

	void readClbk(std::vector<std::byte> &data, const int64_t byteOffset) {
		if (byteOffset >= 0) {
			SimpleReadOnlyFile::seek(static_cast<uint64_t>(byteOffset));
		}

		SimpleReadOnlyFile::read(data);
	}

	void createFileInfo() {
		int64_t lowestTimestamp  = INT64_MAX;
		int64_t highestTimestamp = 0;

		for (const auto &inputDef : mFileInfo.mDataTable.Table) {
			// Timestamp not available is denoted by -1.
			// This cannot happen in files recorded in DV 2.x since we now
			// require timestamps to exist, but in theory older files could
			// have such data types, even if none are known to us.
			if (inputDef.TimestampStart == -1) {
				continue;
			}

			// Determine lowest and highest timestamps present in file.
			if (inputDef.TimestampStart < lowestTimestamp) {
				lowestTimestamp = inputDef.TimestampStart;
			}

			if (inputDef.TimestampEnd > highestTimestamp) {
				highestTimestamp = inputDef.TimestampEnd;
			}

			mFileInfo.mPerStreamDataTables[inputDef.PacketInfo.StreamID()].Table.push_back(inputDef);
		}

		mFileInfo.mTimeLowest  = lowestTimestamp;
		mFileInfo.mTimeHighest = highestTimestamp;

		mFileInfo.mTimeDifference = mFileInfo.mTimeHighest - mFileInfo.mTimeLowest;
		mFileInfo.mTimeShift      = mFileInfo.mTimeLowest;

		mFileInfo.mStreams = mReader.getStreams();
	}

	[[nodiscard]] static dv::cvector<dv::FileDataDefinition>::const_iterator getStartingPointForTimeRangeSearch(
		const int64_t startTimestamp, const dv::FileDataTable &streamDataTable) {
		return std::lower_bound(streamDataTable.Table.cbegin(), streamDataTable.Table.cend(), startTimestamp,
			[](const dv::FileDataDefinition &elem, const int64_t timestamp) {
				return aheadOfRange(timestamp, 0, elem);
			});
	}
};

} // namespace dv::io

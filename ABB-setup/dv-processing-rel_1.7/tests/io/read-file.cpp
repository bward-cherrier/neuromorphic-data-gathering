#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/read_only_file.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	dv::io::ReadOnlyFile testFile{"./test_files/test-minimal.aedat4", dv::io::support::defaultTypeResolver};

	"check_fileinfo"_test = [&] {
		const auto &fileInfo = testFile.getFileInfo();

		expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::LZ4_HIGH)));
		expect(eq(fileInfo.mDataTablePosition, 2009913));
		expect(eq(fileInfo.mDataTableSize, 14545));
		expect(eq(fileInfo.mFileSize, 2024458));
		expect(eq(fileInfo.mStreams.size(), 3));
		expect(eq(fileInfo.mTimeDifference, 2699987));
		expect(eq(fileInfo.mTimeHighest, 1631717224374502));
		expect(eq(fileInfo.mTimeLowest, 1631717221674515));
		expect(eq(fileInfo.mTimeShift, 1631717221674515));
	};

	"read_data_time_range"_test = [&] {
		const auto startTimestamp = testFile.getFileInfo().mTimeLowest;

		// Get 100ms range from start in event-stream (ID=0).
		const auto packets = testFile.read(startTimestamp, startTimestamp + 99'999, 0);

		expect(eq(packets.size(), 10));
		expect(eq(static_cast<const dv::EventPacket *>(packets[0].first->obj)->elements.size(), 967));
		expect(eq(static_cast<const dv::EventPacket *>(packets[1].first->obj)->elements.size(), 973));
		expect(eq(static_cast<const dv::EventPacket *>(packets[2].first->obj)->elements.size(), 957));
	};

	"read_data_time_range_invalid"_test = [&] {
		const int64_t startTimestamp = 10;

		// Get 100ms range from non-existing timestamp range in event-stream (ID=0).
		const auto packets = testFile.read(startTimestamp, startTimestamp + 99'999, 0);

		expect(eq(packets.size(), 0));
	};

	"read_data_stream_id_invalid"_test = [&] {
		const auto startTimestamp = testFile.getFileInfo().mTimeLowest;

		// Get 100ms range from non-existing stream ID=42.
		const auto packets = testFile.read(startTimestamp, startTimestamp + 99'999, 42);

		expect(eq(packets.size(), 0));
	};

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/read_only_file.hpp"
#include "../../include/dv-processing/io/write_only_file.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	dv::io::ReadOnlyFile testFile{"./test_files/test-minimal.aedat4", dv::io::support::defaultTypeResolver};

	"write_compression_none"_test = [&] {
		// Write file.
		{
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}

			dv::io::support::XMLConfigWriter info{root};

			dv::io::WriteOnlyFile outFile{"./out.aedat4", info.getXMLContent(), dv::CompressionType::NONE};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
				}
			}
		}

		// Read back for verification.
		{
			dv::io::ReadOnlyFile outFileCheck{"./out.aedat4", dv::io::support::defaultTypeResolver};

			const auto &fileInfo = outFileCheck.getFileInfo();

			expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::NONE)));
			expect(eq(fileInfo.mStreams.size(), 3));
			expect(eq(fileInfo.mTimeDifference, 2699987));
			expect(eq(fileInfo.mTimeHighest, 1631717224374502));
			expect(eq(fileInfo.mTimeLowest, 1631717221674515));
			expect(eq(fileInfo.mTimeShift, 1631717221674515));
		}
	};

	"write_compression_lz4"_test = [&] {
		// Write file.
		{
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}

			dv::io::support::XMLConfigWriter info{root};

			dv::io::WriteOnlyFile outFile{"./out.aedat4", info.getXMLContent(), dv::CompressionType::LZ4};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
				}
			}
		}

		// Read back for verification.
		{
			dv::io::ReadOnlyFile outFileCheck{"./out.aedat4", dv::io::support::defaultTypeResolver};

			const auto &fileInfo = outFileCheck.getFileInfo();

			expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::LZ4)));
			expect(eq(fileInfo.mStreams.size(), 3));
			expect(eq(fileInfo.mTimeDifference, 2699987));
			expect(eq(fileInfo.mTimeHighest, 1631717224374502));
			expect(eq(fileInfo.mTimeLowest, 1631717221674515));
			expect(eq(fileInfo.mTimeShift, 1631717221674515));
		}
	};

	"write_compression_lz4_custom"_test = [&] {
		// Write file.
		{
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}

			dv::io::support::XMLConfigWriter info{root};

			LZ4F_preferences_t preferences;
			std::memset(&preferences, 0, sizeof(LZ4F_preferences_t));

			// Some custom compression parameters
			preferences.compressionLevel    = 8;
			preferences.frameInfo.blockMode = LZ4F_blockIndependent;

			dv::io::WriteOnlyFile outFile{"./out.aedat4", info.getXMLContent(),
				std::make_unique<dv::io::compression::Lz4CompressionSupport>(preferences)};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
				}
			}
		}

		// Read back for verification, should be agnostic to custom compression configuration
		{
			dv::io::ReadOnlyFile outFileCheck{"./out.aedat4", dv::io::support::defaultTypeResolver};

			const auto &fileInfo = outFileCheck.getFileInfo();

			expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::LZ4)));
			expect(eq(fileInfo.mStreams.size(), 3));
			expect(eq(fileInfo.mTimeDifference, 2699987));
			expect(eq(fileInfo.mTimeHighest, 1631717224374502));
			expect(eq(fileInfo.mTimeLowest, 1631717221674515));
			expect(eq(fileInfo.mTimeShift, 1631717221674515));
		}
	};

	"write_compression_zstd"_test = [&] {
		// Write file.
		{
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}

			dv::io::support::XMLConfigWriter info{root};

			dv::io::WriteOnlyFile outFile{"./out.aedat4", info.getXMLContent(), dv::CompressionType::ZSTD};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
				}
			}
		}

		// Read back for verification.
		{
			dv::io::ReadOnlyFile outFileCheck{"./out.aedat4", dv::io::support::defaultTypeResolver};

			const auto &fileInfo = outFileCheck.getFileInfo();

			expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::ZSTD)));
			expect(eq(fileInfo.mStreams.size(), 3));
			expect(eq(fileInfo.mTimeDifference, 2699987));
			expect(eq(fileInfo.mTimeHighest, 1631717224374502));
			expect(eq(fileInfo.mTimeLowest, 1631717221674515));
			expect(eq(fileInfo.mTimeShift, 1631717221674515));
		}
	};

	"write_compression_zstd_custom"_test = [&] {
		// Write file.
		{
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}

			dv::io::support::XMLConfigWriter info{root};

			// Custom compression level for zstd
			dv::io::WriteOnlyFile outFile{"./out.aedat4", info.getXMLContent(),
				std::make_unique<dv::io::compression::ZstdCompressionSupport>(10)};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
				}
			}
		}

		// Read back for verification, should be agnostic to custom compression configuration
		{
			dv::io::ReadOnlyFile outFileCheck{"./out.aedat4", dv::io::support::defaultTypeResolver};

			const auto &fileInfo = outFileCheck.getFileInfo();

			expect(eq(dv::EnumAsInteger(fileInfo.mCompression), dv::EnumAsInteger(dv::CompressionType::ZSTD)));
			expect(eq(fileInfo.mStreams.size(), 3));
			expect(eq(fileInfo.mTimeDifference, 2699987));
			expect(eq(fileInfo.mTimeHighest, 1631717224374502));
			expect(eq(fileInfo.mTimeLowest, 1631717221674515));
			expect(eq(fileInfo.mTimeShift, 1631717221674515));
		}
	};

	return EXIT_SUCCESS;
}

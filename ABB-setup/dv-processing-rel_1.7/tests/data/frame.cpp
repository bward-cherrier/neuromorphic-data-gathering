#include "../../include/dv-processing/data/frame_base.hpp"
#include "../../include/dv-processing/io/mono_camera_recording.hpp"
#include "../../include/dv-processing/io/read_only_file.hpp"
#include "../../include/dv-processing/io/write_only_file.hpp"

#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

int main() {
	using namespace boost::ut;

	"read_write"_test = [] {
		std::vector<dv::Frame> frames;
		{
			dv::io::ReadOnlyFile testFile{"./test_files/test-frame.aedat4"};
			dv::io::support::XMLTreeNode root{"outInfo"};

			for (const auto &stream : testFile.getFileInfo().mStreams) {
				root.mChildren.emplace_back(stream.mXMLNode);
			}
			dv::io::support::XMLConfigWriter info{root};

			dv::io::WriteOnlyFile outFile{"./out-frame.aedat4", info.getXMLContent(), dv::CompressionType::NONE};

			for (const auto &[streamId, dataTable] : testFile.getFileInfo().mPerStreamDataTables) {
				for (const auto &packetInfo : dataTable.Table) {
					const auto packet = testFile.read(packetInfo);
					outFile.write(packet.first.get(), streamId);
					frames.push_back(*static_cast<dv::Frame *>(packet.first->obj));
				}
			}
		}

		dv::io::MonoCameraRecording reader("./out-frame.aedat4");

		expect(reader.isFrameStreamAvailable());

		size_t index = 0;
		while (auto image = reader.getNextFrame()) {
			expect(lt(index, frames.size()));

			// We should read the same exact images
			expect(eq(cv::sum(image->image - frames[index].image)[0], 0));
			index++;
		}

		expect(eq(index, frames.size()));
	};

	return EXIT_SUCCESS;
}

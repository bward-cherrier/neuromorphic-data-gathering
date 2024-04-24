#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/stereo_camera_recording.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	std::string file = "./test_files/test-minimal.aedat4";

	"read_stereo"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);
		const std::string &cameraName = reader.getCameraName();

		// Emulate stereo by assuming the only camera in the file is the stereo setup
		dv::io::StereoCameraRecording stereoReader(file, cameraName, cameraName);
		expect(eq(stereoReader.getLeftReader().getCameraName(), cameraName));
		expect(eq(stereoReader.getRightReader().getCameraName(), cameraName));
	};

	return EXIT_SUCCESS;
}

#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Get and print the camera name that data from recorded from
	std::cout << "Opened an AEDAT4 file which contains data from [" << reader.getCameraName() << "] camera"
			  << std::endl;

	return 0;
}

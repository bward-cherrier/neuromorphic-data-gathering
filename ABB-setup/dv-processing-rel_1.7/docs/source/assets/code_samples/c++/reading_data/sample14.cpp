#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Store some stream name
	const std::string streamName = "poses";

	// Check if such stream name is available and validate the data type of this stream
	if (reader.isStreamAvailable("streamName") && reader.isStreamOfDataType<dv::Pose>("streamName")) {
		std::cout << "The file contains a stream named [" << streamName << "] and of data type [dv::Pose]" << std::endl;
	}

	return 0;
}

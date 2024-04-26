#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Store the file path
	const std::string pathToFile = "path/to/file.aedat4";

	// Open a file
	dv::io::MonoCameraRecording reader(pathToFile);

	// Print file path and camera name
	std::cout << "Available streams in [" << pathToFile << "]:" << std::endl;

	// Check if event stream is available
	if (reader.isEventStreamAvailable()) {
		// Check the resolution of event stream. Since the getEventResolution() method returns
		// a std::optional, we use *operator to get the value. The method returns std::nullopt
		// only in case the stream is unavailable, which is already checked.
		const cv::Size resolution = *reader.getEventResolution();

		// Print that the stream is present and its resolution
		std::cout << "  * Event stream with resolution " << resolution << std::endl;
	}

	// Check if frame stream is available
	if (reader.isFrameStreamAvailable()) {
		// Check the resolution of frame stream. Since the getFrameResolution() method returns
		// a std::optional, we use *operator to get the value. The method returns std::nullopt
		// only in case the stream is unavailable, which is already checked.
		const cv::Size resolution = *reader.getFrameResolution();

		// Print that the stream is available and its resolution
		std::cout << "  * Frame stream with resolution " << resolution << std::endl;
	}

	// Check if IMU stream is available
	if (reader.isImuStreamAvailable()) {
		// Print that the IMU stream is available
		std::cout << "  * IMU stream" << std::endl;
	}

	// Check if trigger stream is available
	if (reader.isTriggerStreamAvailable()) {
		// Print that the trigger stream is available
		std::cout << "  * Trigger stream " << std::endl;
	}

	return 0;
}

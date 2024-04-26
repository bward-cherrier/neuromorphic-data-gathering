#include <dv-processing/data/timed_keypoint_base.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Define and contain a stream name in a variable
	const std::string stream = "keypoints";

	// Check whether a timed-keypoint stream is available
	if (!reader.isStreamAvailable(stream) || !reader.isStreamOfDataType<dv::TimedKeyPointPacket>(stream)) {
		throw dv::exceptions::RuntimeError("Stream named 'keypoints' not found");
	}

	// Run the loop while data is available
	while (reader.isRunning()) {
		// Read timed keypoint batch, check whether it is correct.
		if (const auto keypoints = reader.getNextStreamPacket<dv::TimedKeyPointPacket>(stream); keypoints.has_value()) {
			// Print the number of keypoints read
			std::cout << "Read " << keypoints->elements.size() << " timed keypoints" << std::endl;
		}
	}

	return 0;
}

#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Run the loop while data is available
	while (reader.isRunning()) {
		// Read batch of events, check whether received data is correct.
		if (const auto events = reader.getNextEventBatch(); events.has_value()) {
			// Print received event packet information
			std::cout << *events << std::endl;
		}
	}

	return 0;
}

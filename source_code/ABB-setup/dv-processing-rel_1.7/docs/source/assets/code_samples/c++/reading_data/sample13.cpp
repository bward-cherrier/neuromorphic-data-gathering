#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Run the loop while data is available
	while (reader.isRunning()) {
		// Read trigger batch, check whether it is correct.
		if (const auto triggers = reader.getNextTriggerBatch(); triggers.has_value() && !triggers->empty()) {
			// Print the trigger batch information
			std::cout << "Received " << triggers->size() << " triggers" << std::endl;
		}
	}

	return 0;
}

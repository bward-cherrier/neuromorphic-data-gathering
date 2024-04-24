#include <dv-processing/io/mono_camera_recording.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Run the loop while data is available
	while (reader.isRunning()) {
		// Read IMU measurement batch, check whether it is correct.
		if (const auto imuBatch = reader.getNextImuBatch(); imuBatch.has_value() && !imuBatch->empty()) {
			// Print IMU batch information
			std::cout << "Received " << imuBatch->size() << " IMU measurements" << std::endl;
		}
	}

	return 0;
}

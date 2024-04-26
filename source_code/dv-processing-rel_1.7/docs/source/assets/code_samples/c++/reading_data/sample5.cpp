#include <dv-processing/io/camera_capture.hpp>

#include <chrono>

int main() {
	using namespace std::chrono_literals;

	// Open any camera
	dv::io::CameraCapture capture;

	// Run the loop while camera is still connected
	while (capture.isRunning()) {
		// Read IMU measurement batch, check whether it is correct.
		// The method does not wait for data to arrive, it returns immediately with
		// the latest available imu data or if no data is available, returns a `std::nullopt`.
		if (const auto imuBatch = capture.getNextImuBatch(); imuBatch.has_value() && !imuBatch->empty()) {
			std::cout << "Received " << imuBatch->size() << " IMU measurements" << std::endl;
		}
		else {
			// No data has arrived yet, short sleep to reduce CPU load.
			std::this_thread::sleep_for(1ms);
		}
	}

	return 0;
}

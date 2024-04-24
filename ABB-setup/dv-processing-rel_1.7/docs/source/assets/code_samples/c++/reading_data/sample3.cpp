#include <dv-processing/io/camera_capture.hpp>

#include <chrono>

int main() {
	using namespace std::chrono_literals;

	// Open any camera
	dv::io::CameraCapture capture;

	// Run the loop while camera is still connected
	while (capture.isRunning()) {
		// Read batch of events, check whether received data is correct.
		// The method does not wait for data arrive, it returns immediately with
		// the latest available data or if no data is available, returns a `std::nullopt`.
		if (const auto events = capture.getNextEventBatch(); events.has_value()) {
			// Print received packet information
			std::cout << *events << std::endl;
		}
		else {
			// No data has arrived yet, short sleep to reduce CPU load.
			std::this_thread::sleep_for(1ms);
		}
	}

	return 0;
}

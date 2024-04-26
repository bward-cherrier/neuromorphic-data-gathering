#include <dv-processing/io/camera_capture.hpp>

#include <chrono>

int main() {
	using namespace std::chrono_literals;

	// Open any camera
	dv::io::CameraCapture capture;

	// Depending on the incoming signal, enable the detection of the desired type of pattern, here we enable everything.
	// Note: In the following variables, replace 'DVX' with 'DAVIS_CONFIG' in case the device used is a DAVIS.
	// Enable rising edge detection
	capture.deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_RISING_EDGES, true);
	// Enable falling edge detection
	capture.deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_FALLING_EDGES, true);
	// Enable pulse detection
	capture.deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_PULSES, true);
	// Enable detector
	capture.deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_RUN_DETECTOR, true);

	// Run the loop while camera is still connected
	while (capture.isRunning()) {
		// Read trigger batch, check whether it is correct.
		// The method does not wait for data to arrive, it returns immediately with
		// the latest available data or if no data is available, returns a `std::nullopt`.
		if (const auto triggers = capture.getNextTriggerBatch(); triggers.has_value() && !triggers->empty()) {
			std::cout << "Received " << triggers->size() << " Triggers" << std::endl;
		}
		else {
			// No data has arrived yet, short sleep to reduce CPU load.
			std::this_thread::sleep_for(1ms);
		}
	}

	return 0;
}

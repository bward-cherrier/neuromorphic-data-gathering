#include <dv-processing/io/camera_capture.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Print the camera name
	std::cout << "Opened [" << capture.getCameraName() << "] camera, it provides:" << std::endl;

	// Check whether event stream is available
	if (capture.isEventStreamAvailable()) {
		// Get the event stream resolution, the output is a std::optional, so the value() method is
		// used to get the actual resolution value
		const cv::Size resolution = capture.getEventResolution().value();

		// Print the event stream capability with resolution value
		std::cout << "* Events at " << resolution << " resolution" << std::endl;
	}

	// Check whether frame stream is available
	if (capture.isFrameStreamAvailable()) {
		// Get the frame stream resolution
		const cv::Size resolution = capture.getFrameResolution().value();

		// Print the frame stream capability with resolution value
		std::cout << "* Frames at " << resolution << " resolution" << std::endl;
	}

	// Check whether the IMU stream is available
	if (capture.isImuStreamAvailable()) {
		// Print the imu data stream capability
		std::cout << "* IMU measurements" << std::endl;
	}

	// Check whether the trigger stream is available
	if (capture.isTriggerStreamAvailable()) {
		// Print the trigger stream capability
		std::cout << "* Triggers" << std::endl;
	}

	return 0;
}

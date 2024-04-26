#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Create the writer instance, writer will inspect the capture capabilities and create output
	// streams for all available data streams from the capture instance.
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", capture);

	// Print which streams were configured
	std::cout << std::boolalpha;
	std::cout << "Is event stream available? " << writer.isEventStreamConfigured() << std::endl;
	std::cout << "Is frame stream available? " << writer.isFrameStreamConfigured() << std::endl;
	std::cout << "Is imu stream available? " << writer.isImuStreamConfigured() << std::endl;
	std::cout << "Is trigger stream available? " << writer.isTriggerStreamConfigured() << std::endl;

	return 0;
}

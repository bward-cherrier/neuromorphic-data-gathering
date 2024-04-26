#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	dv::io::MonoCameraWriter::Config config("DVXplorer_sample");

	// Sample VGA resolution, same as the DVXplorer camera
	const cv::Size resolution(640, 480);

	// Add an event stream with a resolution
	config.addEventStream(resolution);

	// Add frame stream with a resolution
	config.addFrameStream(resolution);

	// Add IMU stream
	config.addImuStream();

	// Add trigger stream
	config.addTriggerStream();

	// Create the writer instance with the configuration structure
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Print which streams were configured
	std::cout << std::boolalpha;
	std::cout << "Is event stream available? " << writer.isEventStreamConfigured() << std::endl;
	std::cout << "Is frame stream available? " << writer.isFrameStreamConfigured() << std::endl;
	std::cout << "Is imu stream available? " << writer.isImuStreamConfigured() << std::endl;
	std::cout << "Is trigger stream available? " << writer.isTriggerStreamConfigured() << std::endl;

	return 0;
}

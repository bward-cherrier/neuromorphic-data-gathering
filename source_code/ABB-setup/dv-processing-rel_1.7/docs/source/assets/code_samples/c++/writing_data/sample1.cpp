#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Create a DVS config - events, imu, and triggers are going to be enabled
	const auto config = dv::io::MonoCameraWriter::DVSConfig("DVXplorer_sample", cv::Size(640, 480));

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

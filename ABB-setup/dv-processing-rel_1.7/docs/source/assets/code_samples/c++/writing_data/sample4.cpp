#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Sample VGA resolution, same as the DVXplorer camera
	const cv::Size resolution(640, 480);

	// Frame only configuration
	const auto config = dv::io::MonoCameraWriter::FrameOnlyConfig("DVXplorer_sample", resolution);

	// Create the writer instance, it will only have a single frame output stream.
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Write 10 image frames
	for (int i = 0; i < 10; i++) {
		// Initialize a white image
		cv::Mat image(resolution, CV_8UC3, cv::Scalar(255, 255, 255));

		// Generate some monotonically increasing timestamp
		const int64_t timestamp = i * 1000;

		// Encapsulate the image in a frame that has a timestamp, this does not copy the pixel data
		dv::Frame frame(timestamp, image);

		// Write the frame
		writer.writeFrame(frame);
	}

	return 0;
}

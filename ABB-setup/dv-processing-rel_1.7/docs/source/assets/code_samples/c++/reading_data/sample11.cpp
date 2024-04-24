#include <dv-processing/io/mono_camera_recording.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Open a file
	dv::io::MonoCameraRecording reader("path/to/file.aedat4");

	// Initiate a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Variable to store the previous frame timestamp for correct playback
	std::optional<int64_t> lastTimestamp = std::nullopt;

	// Run the loop while data is available
	while (reader.isRunning()) {
		// Read a frame, check whether it is correct.
		// The method does not wait for frame arrive, it returns immediately with
		// latest available frame or if no data is available, returns a `std::nullopt`.
		if (const auto frame = reader.getNextFrame(); frame.has_value()) {
			// Print information about received frame
			std::cout << *frame << std::endl;

			// Show a preview of the image
			cv::imshow("Preview", frame->image);

			// Calculate the delay between last and current frame, divide by 1000 to convert microseconds
			// to milliseconds
			const int delay = lastTimestamp.has_value() ? (frame->timestamp - *lastTimestamp) / 1000 : 2;

			// Perform the sleep
			cv::waitKey(delay);

			// Store timestamp for the next frame
			lastTimestamp = frame->timestamp;
		}
	}

	return 0;
}

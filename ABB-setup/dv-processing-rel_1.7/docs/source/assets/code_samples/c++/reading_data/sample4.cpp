#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Initiate a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Run the loop while camera is still connected
	while (capture.isRunning()) {
		// Read a frame, check whether it is correct.
		// The method does not wait for frame arrive, it returns immediately with
		// the latest available frame or if no data is available, returns a `std::nullopt`.
		if (const auto frame = capture.getNextFrame(); frame.has_value()) {
			std::cout << *frame << std::endl;

			// Show a preview of the image
			cv::imshow("Preview", frame->image);
		}
		cv::waitKey(2);
	}

	return 0;
}

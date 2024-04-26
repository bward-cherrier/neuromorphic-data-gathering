#include <dv-processing/features/feature_tracks.hpp>
#include <dv-processing/features/image_feature_lk_tracker.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Make sure it supports event stream output, throw an error otherwise
	if (!capture.isFrameStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide a frame stream.");
	}

	const cv::Size resolution = capture.getFrameResolution().value();

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Instantiate a visual tracker with known resolution, all parameters kept default
	auto tracker = dv::features::ImageFeatureLKTracker::RegularTracker(resolution);

	// Create a track container instance that is used to visualize tracks on an image
	dv::features::FeatureTracks tracks;

	// Run the frame processing while the camera is connected
	while (capture.isRunning()) {
		// Try to receive a frame, check if anything was received
		if (const auto frame = capture.getNextFrame()) {
			// Pass the frame to the tracker
			tracker->accept(*frame);

			// Run tracking
			const auto result = tracker->runTracking();

			// Pass tracking result into the track container which aggregates track history
			tracks.accept(result);

			// Generate and show a preview of recent tracking history
			cv::imshow("Preview", tracks.visualize(frame->image));
		}
		cv::waitKey(2);
	}

	return 0;
}

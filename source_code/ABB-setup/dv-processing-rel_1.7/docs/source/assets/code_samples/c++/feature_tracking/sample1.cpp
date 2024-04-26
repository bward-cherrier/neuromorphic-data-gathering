#include <dv-processing/features/event_feature_lk_tracker.hpp>
#include <dv-processing/features/feature_tracks.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Make sure it supports event stream output, throw an error otherwise
	if (!capture.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide an event stream.");
	}

	const cv::Size resolution = capture.getEventResolution().value();

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Instantiate a visual tracker with known resolution, all parameters kept default
	auto tracker = dv::features::EventFeatureLKTracker<>::RegularTracker(resolution);

	// Run tracking by accumulating frames with 100 FPS
	tracker->setFramerate(100);

	// Create a track container instance that is used to visualize tracks on an image
	dv::features::FeatureTracks tracks;

	// Run the frame processing while the camera is connected
	while (capture.isRunning()) {
		// Try to receive a batch of events, check if anything was received
		if (const auto events = capture.getNextEventBatch()) {
			// Pass the frame to the tracker
			tracker->accept(*events);

			// Run tracking
			const auto result = tracker->runTracking();

			// Since we are passing events in fine-grained batches, tracking will not execute
			// until enough events is received, returning invalid pointer if tracking did not execute
			if (!result) {
				continue;
			}

			// Pass tracking result into the track container which aggregates track history
			tracks.accept(result);

			// Generate and show a preview of recent tracking history
			cv::imshow("Preview", tracks.visualize(tracker->getAccumulatedFrame()));
		}
		cv::waitKey(2);
	}

	return 0;
}

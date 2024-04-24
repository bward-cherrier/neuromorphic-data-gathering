#include <dv-processing/features/event_combined_lk_tracker.hpp>
#include <dv-processing/features/feature_tracks.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Open any camera
	dv::io::CameraCapture capture;

	// Make sure it supports correct stream outputs, throw an error otherwise
	if (!capture.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide an event stream.");
	}
	if (!capture.isFrameStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide a frame stream.");
	}

	const cv::Size resolution = capture.getEventResolution().value();

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Instantiate a visual tracker with known resolution, all parameters kept default
	auto tracker = dv::features::EventCombinedLKTracker<>::RegularTracker(resolution);

	// Accumulate and track on 5 intermediate accumulated frames between each actual frame pair
	tracker->setNumIntermediateFrames(5);

	// Create a track container instance that is used to visualize tracks on an image
	dv::features::FeatureTracks tracks;

	// Use a queue to store incoming frames to make sure the all data has arrived prior to running the tracking
	std::queue<dv::Frame> frameQueue;

	// Run the frame processing while the camera is connected
	while (capture.isRunning()) {
		// Try to receive a frame, check if anything was received
		if (const auto frame = capture.getNextFrame()) {
			// Push the received frame into the frame queue
			frameQueue.push(*frame);
		}

		// Try to receive a batch of events, check if anything was received
		if (const auto events = capture.getNextEventBatch()) {
			// Pass the frame to the tracker
			tracker->accept(*events);

			// Check if we have ready frames and if enough events have arrived already
			if (frameQueue.empty() || frameQueue.front().timestamp > events->getHighestTime()) {
				continue;
			}

			// Take the last frame from the queue
			const auto frame = frameQueue.front();

			// Pass it to the tracker as well
			tracker->accept(frame);

			// Remove the last used frame from the queue
			frameQueue.pop();

			// Run tracking
			const auto result = tracker->runTracking();

			// Validate that the tracking was successful
			if (!result) {
				continue;
			}

			// Pass tracking result into the track container which aggregates track history
			tracks.accept(result);

			// Generate and show a preview of recent tracking history on both accumulated frames and the frame image
			// Take the set of intermediate accumulated frames from the tracker
			const auto accumulatedFrames = tracker->getAccumulatedFrames();
			if (!accumulatedFrames.empty()) {
				cv::Mat preview;
				// Draw visualization on both image and concatenate them horizontally
				cv::hconcat(
					tracks.visualize(accumulatedFrames.back().pyramid.front()), tracks.visualize(frame.image), preview);
				// Show the final preview image
				cv::imshow("Preview", preview);
			}
		}

		cv::waitKey(2);
	}

	return 0;
}

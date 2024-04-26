#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/io/stereo_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

#include <csignal>

static std::atomic<bool> keepRunning(true);

static void handleShutdown(int) {
	keepRunning.store(false);
}

int main() {
	using namespace std::chrono_literals;

	// Keycode for the escape key
	static constexpr int ESC_KEYCODE = 27;

	// Install signal handlers for a clean shutdown
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	const auto cameras = dv::io::discoverDevices();
	if (cameras.size() < 2) {
		throw dv::exceptions::RuntimeError("Unable to discover two cameras");
	}

	// Open the cameras
	dv::io::StereoCapture stereo(cameras[0], cameras[1]);

	// Initialize a stereo stream slicer
	dv::StereoEventStreamSlicer slicer;

	// Create windows for a display
	cv::namedWindow("Left", cv::WINDOW_NORMAL);
	cv::namedWindow("Right", cv::WINDOW_NORMAL);

	// Initialize visualizers for each camera (we need two because cameras might have
	// different resolutions).
	dv::visualization::EventVisualizer leftVis(stereo.left.getEventResolution().value());
	dv::visualization::EventVisualizer rightVis(stereo.right.getEventResolution().value());

	// Register a job to be performed every 33 milliseconds
	slicer.doEveryTimeInterval(
		// Here we receive events from two camera, time-synchronized
		33ms, [&leftVis, &rightVis](const dv::EventStore &leftEvents, const dv::EventStore &rightEvents) {
			// Perform visualization and show preview
			cv::imshow("Left", leftVis.generateImage(leftEvents));
			cv::imshow("Right", rightVis.generateImage(rightEvents));

			// Signal exit if ESC key is pressed
			if (cv::waitKey(2) == ESC_KEYCODE) {
				keepRunning.store(false);
			}
		});

	while (keepRunning) {
		dv::EventStore left, right;

		// Handle left camera events
		if (const auto batch = stereo.left.getNextEventBatch(); batch.has_value()) {
			left = *batch;
		}

		// Handle right camera events
		if (const auto batch = stereo.right.getNextEventBatch(); batch.has_value()) {
			right = *batch;
		}
		// Pass all events into the slicer
		slicer.accept(left, right);
	}
	return EXIT_SUCCESS;
}

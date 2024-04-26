#include <dv-processing/core/frame.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

#include <csignal>

static std::atomic<bool> globalShutdown(false);

static void handleShutdown(int) {
	globalShutdown.store(true);
}

int main() {
	using namespace std::chrono_literals;

	static constexpr int ESC_KEYCODE = 27;

	// Install signal handlers for a clean shutdown
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	// Open any camera attached to the system
	dv::io::CameraCapture capture;

	std::cout << "Camera [" << capture.getCameraName() << "] has been opened!" << std::endl;

	// Create an accumulator for event images
	dv::EdgeMapAccumulator acc(capture.getEventResolution().value());

	// Create a preview window
	cv::namedWindow("preview", cv::WINDOW_NORMAL);

	// Create an event stream slicer, this is going to be used for event only cameras
	dv::EventStreamSlicer slicer;

	// Slice the events every 33 millisenconds
	slicer.doEveryTimeInterval(33ms, [&](const dv::EventStore &slice) {
		// Accept events
		acc.accept(slice);

		// Generate an accumulated image
		dv::Frame accumulatedFrame = acc.generateFrame();

		// Display preview
		cv::imshow("preview", accumulatedFrame.image);
		if (cv::waitKey(33) == ESC_KEYCODE) {
			globalShutdown.store(true);
		}
	});

	while (!globalShutdown && capture.isRunning()) {
		dv::EventStore latestEvent;
		// Read incoming events
		if (auto batch = capture.getNextEventBatch(); batch.has_value()) {
			latestEvent = std::move(*batch);
		}
		else {
			// No events available yet, short sleep
			std::this_thread::sleep_for(100us);
			continue;
		}

		// Test whether camera support frames as output
		if (capture.isFrameStreamAvailable()) {
			// If so, directly feed the event batch into accumulator
			acc.accept(latestEvent);

			// Test if we received a frame
			if (auto frame = capture.getNextFrame(); frame.has_value()) {
				cv::Mat preview;

				// Generate event accumulated frame
				dv::Frame accumulatedFrame = acc.generateFrame();

				// If the image is color, convert the accumulated image to color as well for concatenation
				if (frame->image.channels() == 3) {
					cv::cvtColor(accumulatedFrame.image, accumulatedFrame.image, cv::COLOR_GRAY2BGR);
				}

				// Concat the two images for preview
				cv::hconcat(frame->image, accumulatedFrame.image, preview);
				cv::imshow("preview", preview);
				if (cv::waitKey(2) == ESC_KEYCODE) {
					globalShutdown.store(true);
				}
			}
		}
		else {
			// Otherwise, use event stream slicer to show the preview
			slicer.accept(latestEvent);
		}
	}

	return EXIT_SUCCESS;
}

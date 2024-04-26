#include <dv-processing/core/multi_stream_slicer.hpp>
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
	dv::PixelAccumulator acc(capture.getEventResolution().value());

	// Create a preview window
	cv::namedWindow("Events", cv::WINDOW_NORMAL);

	// Data read handler is a utility class to provide correct type handlers to each class
	dv::io::DataReadHandler handler;

	// Create an event stream slicer, this is going to be used for event only cameras
	dv::MultiStreamSlicer<dv::EventStore> slicer("events");

	// Add handler that passes events from camera to slicer as the main stream
	handler.mEventHandler = [&slicer](const auto &events) {
		slicer.accept("events", events);
	};

	// Handle IMU data
	slicer.addStream<dv::cvector<dv::IMU>>("imu");
	handler.mImuHandler = [&slicer](const auto &imu) {
		slicer.accept("imu", imu);
	};

	// Handle trigger data
	slicer.addStream<dv::cvector<dv::Trigger>>("triggers");
	handler.mTriggersHandler = [&slicer](const auto &triggers) {
		slicer.accept("triggers", triggers);
	};

	// Handle frames as well, if the camera supports them
	if (capture.isFrameStreamAvailable()) {
		slicer.addStream<dv::cvector<dv::Frame>>("frames");
		handler.mFrameHandler = [&slicer](const auto &frame) {
			slicer.accept("frames", frame);
		};
		// Initialize an empty frame for a smooth preview
		cv::namedWindow("Frames", cv::WINDOW_NORMAL);
	}

	// Slice the data every 33 milliseconds
	slicer.doEveryTimeInterval(33ms, [&](const auto &data) {
		// Accept events
		acc.accept(data.template get<dv::EventStore>("events"));
		// Generate an accumulated image
		dv::Frame accumulatedFrame = acc.generateFrame();
		// Display preview
		cv::imshow("Events", accumulatedFrame.image);

		// If we have frames
		if (data.contains("frames")) {
			const auto frames = data.template get<dv::cvector<dv::Frame>>("frames");
			if (!frames.empty()) {
				// Display latest available frame
				cv::imshow("Frames", frames.back().image);
			}
		}
		if (cv::waitKey(2) == ESC_KEYCODE) {
			globalShutdown.store(true);
		}
	});

	while (!globalShutdown && capture.handleNext(handler)) {
		// Triggers do not generate a periodic signal, so synchronizing against it becomes impossible, since
		// amount of arrived data is unknown. This is handled by manually passing latest event stream timestamp from
		// the camera capture
		slicer.setStreamSeekTime("triggers", capture.getEventSeekTime());
		usleep(1000);
	}

	return EXIT_SUCCESS;
}

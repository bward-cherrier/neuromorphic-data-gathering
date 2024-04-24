#include <dv-processing/core/multi_stream_slicer.hpp>
#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
	// Open a DAVIS camera, the sample will slice incoming frames and events synchronously using the generic data
	// stream slicer. The camera name is an empty string to open any DAVIS camera detected on the system
	dv::io::CameraCapture camera("", dv::io::CameraCapture::CameraType::DAVIS);

	// Declare the main stream type to be dv::cvector<dv::Frame> with stream name "frames"
	dv::MultiStreamSlicer<dv::cvector<dv::Frame>> slicer("frames");

	// Add a secondary stream of events
	slicer.addStream<dv::EventStore>("events");

	// It's possible to add additional secondary streams for slicing here, e.g. adding a trigger stream
	// would look like:
	// slicer.addStream<dv::TriggerPacket>("triggers");

	// Use visualizer to overlay events on a frame
	dv::visualization::EventVisualizer visualizer(camera.getEventResolution().value(), dv::visualization::colors::white,
		dv::visualization::colors::green, dv::visualization::colors::red);

	// Declare display windows for frames
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Register a job to be performed every frame, the event will be sliced in-between the main stream frames
	slicer.doEveryNumberOfElements(
		// Here we receive time-synchronized frames and events; sliced data is provided in a std::unordered_map
		// for easy access using the stream name.
		1, [&visualizer](const auto &data) {
			// Extract events and pass them to the slicer, data is retrieved using the helper method `get()`
			// which accepts the stream name and the type. Stream name and type has to match, otherwise
			// an exception will be thrown
			const auto events = data.template get<dv::EventStore>("events");

			// Retrieve frames, although we get one frame per slice, it is stored in the configured container
			const auto frames = data.template get<dv::cvector<dv::Frame>>("frames");

			// The container is non-empty, we expect only one frame in the container, so just display
			// the first frame in the container
			const dv::Frame &frame = frames.at(0);

			// Convert the frame into a grayscale image for overlay preview
			cv::Mat preview;
			if (frame.image.channels() == 3) {
				// The image is already grayscale, no conversion is needed
				preview = frame.image;
			}
			else {
				// The image coming from the camera is multichannel image, so we can safely assume
				// it's a grayscale image, convert it into BGR for overlay drawing
				cv::cvtColor(frame.image, preview, cv::COLOR_GRAY2BGR);
			}

			// Overlay events on top of preview image
			visualizer.generateImage(events, preview);

			// Display the overlayed image
			cv::imshow("Preview", preview);

			// If escape button is pressed (code 27 is escape key), exit the program cleanly
			if (cv::waitKey(2) == 27) {
				exit(0);
			}
		});

	// Continue the loop while both cameras are connected
	while (camera.isRunning()) {
		// Handle events
		if (const auto events = camera.getNextEventBatch(); events.has_value()) {
			slicer.accept("events", *events);
		}

		// Handle frames
		if (const auto frame = camera.getNextFrame(); frame.has_value()) {
			slicer.accept("frames", *frame);
		}
	}

	return 0;
}

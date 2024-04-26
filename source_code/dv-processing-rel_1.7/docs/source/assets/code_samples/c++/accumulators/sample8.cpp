#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Open any camera
	dv::io::CameraCapture capture;

	// Make sure it supports event stream output, throw an error otherwise
	if (!capture.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide an event stream.");
	}

	// Initialize an accumulator with some resolution
	dv::visualization::EventVisualizer visualizer(*capture.getEventResolution());

	// Apply color scheme configuration, these values can be modified to taste
	visualizer.setBackgroundColor(dv::visualization::colors::white);
	visualizer.setPositiveColor(dv::visualization::colors::iniBlue);
	visualizer.setNegativeColor(dv::visualization::colors::darkGrey);

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Initialize a slicer
	dv::EventStreamSlicer slicer;

	// Register a callback every 33 milliseconds
	slicer.doEveryTimeInterval(33ms, [&visualizer](const dv::EventStore &events) {
		// Generate a preview frame
		cv::Mat image = visualizer.generateImage(events);

		// Show the accumulated image
		cv::imshow("Preview", image);
		cv::waitKey(2);
	});

	// Run the event processing while the camera is connected
	while (capture.isRunning()) {
		// Receive events, check if anything was received
		if (const auto events = capture.getNextEventBatch()) {
			// If so, pass the events into the slicer to handle them
			slicer.accept(*events);
		}
	}

	return 0;
}

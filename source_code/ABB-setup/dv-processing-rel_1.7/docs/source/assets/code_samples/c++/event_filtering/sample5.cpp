#include <dv-processing/core/filters.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	const cv::Size resolution(200, 200);

	// Initializing input events with events that represent a logo
	dv::EventStore events = dv::data::generate::dvLogoAsEvents(0, resolution);

	// Initialize region filter using hardcoded coordinates
	dv::EventRegionFilter filter(cv::Rect(50, 50, 100, 100));

	// Pass events to the filter
	filter.accept(events);

	// Call generate events to apply the filter
	const dv::EventStore filtered = filter.generateEvents();

	// Use a visualizer instance to preview the events
	dv::visualization::EventVisualizer visualizer(resolution);

	// Generate preview images of data input and output
	const cv::Mat input  = visualizer.generateImage(events);
	const cv::Mat output = visualizer.generateImage(filtered);

	// Concatenate the images into a single image for preview
	cv::Mat preview;
	cv::hconcat(input, output, preview);

	// Display the input and output images
	cv::namedWindow("preview", cv::WINDOW_NORMAL);
	cv::imshow("preview", preview);
	cv::waitKey();

	return 0;
}

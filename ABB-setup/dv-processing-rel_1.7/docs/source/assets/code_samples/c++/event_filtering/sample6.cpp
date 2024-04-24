#include <dv-processing/core/filters.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	const cv::Size resolution(200, 200);

	// Initializing input events with events that represent a logo
	dv::EventStore events = dv::data::generate::dvLogoAsEvents(0, resolution);

	// Initialize event filter chain, it contains no filters
	dv::EventFilterChain filter;

	// Now let's add filters
	// First, add a region filter with hardcoded coordinates
	filter.addFilter(std::make_shared<dv::EventRegionFilter<>>(cv::Rect(50, 50, 100, 100)));

	// Second, add a positive polarity filter
	filter.addFilter(std::make_shared<dv::EventPolarityFilter<>>(true));

	// Third, add a background activity noise filter
	filter.addFilter(std::make_shared<dv::noise::BackgroundActivityNoiseFilter<>>(resolution));

	// Pass events to the filter
	filter.accept(events);

	// Call generate events to apply the filter chain, it will apply all three filters
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

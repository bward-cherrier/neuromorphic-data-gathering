#include <dv-processing/data/generate.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Hardcoded VGA resolution
	const cv::Size resolution(640, 480);

	// Initializing input events with uniformly distributed events which represent noise
	dv::EventStore events = dv::data::generate::uniformEventsWithinTimeRange(0, 10ms, resolution, 1000);

	// Adding additional data for drawing, this will give an idea whether the filter removes actual signal events
	events.add(dv::data::generate::dvLogoAsEvents(10000, resolution));

	// Initialize a background activity noise filter with 1-millisecond activity period
	dv::noise::BackgroundActivityNoiseFilter filter(resolution, 1ms);

	// Pass events to the filter
	filter.accept(events);

	// Call generate events to apply the noise filter
	const dv::EventStore filtered = filter.generateEvents();

	// Print out the reduction factor, which indicates the percentage of discarded events
	std::cout << "Filter reduced number of events by a factor of " << filter.getReductionFactor() << std::endl;

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

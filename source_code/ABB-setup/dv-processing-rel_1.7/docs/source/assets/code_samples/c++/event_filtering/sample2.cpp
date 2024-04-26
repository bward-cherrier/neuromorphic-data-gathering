#include <dv-processing/core/filters.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Smaller resolution for previews
	const cv::Size resolution(200, 200);

	// Initializing input events with events that represent a logo
	const dv::EventStore events = dv::data::generate::dvLogoAsEvents(0, resolution);

	// Initialize a mask with all zero values
	cv::Mat mask(resolution, CV_8UC1, cv::Scalar(0));

	// Draw two rectangles to generate a similar to checkerboard mask pattern
	cv::rectangle(
		mask, cv::Point(0, 0), cv::Point(resolution.width / 2, resolution.height / 2), cv::Scalar(255), cv::FILLED);
	cv::rectangle(mask, cv::Point(resolution.width / 2, resolution.height / 2),
		cv::Point(resolution.width, resolution.height), cv::Scalar(255), cv::FILLED);

	// Initialize the mask filter with the generated mask
	dv::EventMaskFilter filter(mask);

	// Pass events to the filter
	filter.accept(events);

	// Call generate events to apply the filter
	const dv::EventStore filtered = filter.generateEvents();

	// Print out the reduction factor, which indicates the percentage of discarded events
	std::cout << "Filter reduced number of events by a factor of " << filter.getReductionFactor() << std::endl;

	// Use a visualizer instance to preview the events
	dv::visualization::EventVisualizer visualizer(resolution);

	// Generate preview images of data input and output
	const cv::Mat input  = visualizer.generateImage(events);
	const cv::Mat output = visualizer.generateImage(filtered);

	// Concatenate the images into a single image for preview
	cv::Mat preview, maskColored;
	cv::cvtColor(mask, maskColored, cv::COLOR_GRAY2BGR);
	cv::hconcat(std::vector<cv::Mat>({input, maskColored, output}), preview);

	// Display the input and output images
	cv::namedWindow("preview", cv::WINDOW_NORMAL);
	cv::imshow("preview", preview);
	cv::waitKey();

	return 0;
}

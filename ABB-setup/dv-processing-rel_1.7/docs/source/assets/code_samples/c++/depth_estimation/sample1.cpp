#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/cluster/mean_shift/event_store_adaptor.hpp>
#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/data/utilities.hpp>
#include <dv-processing/depth/sparse_event_block_matcher.hpp>
#include <dv-processing/io/stereo_capture.hpp>
#include <dv-processing/visualization/colors.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Path to a stereo calibration file, replace with a file path on your local file system
	const std::string calibrationFilePath = "path/to/calibration.json";

	// Load the calibration file
	auto calibration = dv::camera::CalibrationSet::LoadFromFile(calibrationFilePath);

	// It is expected that calibration file will have "C0" as the leftEventBuffer camera
	auto leftCamera = calibration.getCameraCalibration("C0").value();

	// The second camera is assumed to be rightEventBuffer-side camera
	auto rightCamera = calibration.getCameraCalibration("C1").value();

	// Open the stereo camera with camera names from calibration
	dv::io::StereoCapture capture(leftCamera.name, rightCamera.name);

	// Make sure both cameras support event stream output, throw an error otherwise
	if (!capture.left.isEventStreamAvailable() || !capture.right.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide an event stream.");
	}

	// Matching window size for the block matcher
	const cv::Size window(24, 24);
	// Minimum disparity value to measure
	const int minDisparity = 0;
	// Maximum disparity value
	const int maxDisparity = 40;
	// Minimum z-score value that a valid match can have
	const float minScore = 0.0f;

	// Initialize the block matcher with rectification
	auto matcher = dv::SparseEventBlockMatcher(std::make_unique<dv::camera::StereoGeometry>(leftCamera, rightCamera),
		window, maxDisparity, minDisparity, minScore);

	// Initialization of a stereo event sliver
	dv::StereoEventStreamSlicer slicer;

	// Initialize a window to show previews of the output
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Local event buffers to implement overlapping window of events for accumulation
	dv::EventStore leftEventBuffer, rightEventBuffer;

	// Use one third of the resolution as count of events per accumulated frame
	const size_t eventCount = static_cast<size_t>(leftCamera.resolution.area()) / 3;

	// Register a callback to be done at 50Hz
	slicer.doEveryTimeInterval(20ms, [&matcher, &leftEventBuffer, &rightEventBuffer, eventCount, &window](
										 const auto &leftEvents, const auto &rightEvents) {
		// Push input events into the local buffers
		leftEventBuffer.add(leftEvents);
		rightEventBuffer.add(rightEvents);

		// If the number of events is above the count, just keep the latest events
		if (leftEventBuffer.size() > eventCount) {
			leftEventBuffer = leftEventBuffer.sliceBack(eventCount);
		}
		if (rightEventBuffer.size() > eventCount) {
			rightEventBuffer = rightEventBuffer.sliceBack(eventCount);
		}

		// Number of clusters to extract
		constexpr int numClusters = 100;

		// Initialize the mean-shift clustering algorithm
		dv::cluster::mean_shift::MeanShiftEventStoreAdaptor meanShift(leftEventBuffer, 10.f, 1.0f, 20, numClusters);

		// Find cluster centers which are going to be used for disparity estimation
		auto centers = meanShift.findClusterCentres<dv::cluster::mean_shift::kernel::Epanechnikov>();

		// Run disparity estimation, the output will contain a disparity estimate for each of the given points.
		const std::vector<dv::SparseEventBlockMatcher::PixelDisparity> estimates
			= matcher.computeDisparitySparse(leftEventBuffer, rightEventBuffer, dv::data::convertToCvPoints(centers));

		// Convert the accumulated frames into colored images for preview.
		std::vector<cv::Mat> images(2);
		cv::cvtColor(matcher.getLeftFrame().image, images[0], cv::COLOR_GRAY2BGR);
		cv::cvtColor(matcher.getRightFrame().image, images[1], cv::COLOR_GRAY2BGR);

		// Visualize the matched blocks
		int32_t index = 0;
		for (const auto &point : estimates) {
			// If point estimation is invalid, do not show a preview of it
			if (!point.valid) {
				continue;
			}

			// The rest of the code performs drawing of the match according to the disparity value on the
			// preview images.
			const cv::Scalar color = dv::visualization::colors::someNeonColor(index++);
			// Draw some nice colored markers and rectangles.
			cv::drawMarker(images[1], *point.matchedPosition, color, cv::MARKER_CROSS, 7);
			cv::rectangle(images[1],
				cv::Rect(point.matchedPosition->x - (window.width / 2), point.matchedPosition->y - (window.height / 2),
					window.width, window.height),
				color);
			cv::rectangle(images[0],
				cv::Rect(point.templatePosition->x - (window.width / 2),
					point.templatePosition->y - (window.height / 2), window.width, window.height),
				color);
		}

		// Concatenate images and show them in a window
		cv::Mat preview;
		cv::hconcat(images, preview);
		cv::imshow("Preview", preview);
	});

	// Buffer input events in these variables to synchronize inputs
	std::optional<dv::EventStore> leftEvents  = std::nullopt;
	std::optional<dv::EventStore> rightEvents = std::nullopt;

	// Run the processing loop while both cameras are connected
	while (capture.left.isRunning() && capture.right.isRunning()) {
		// Read events from respective left / right cameras
		if (!leftEvents.has_value()) {
			leftEvents = capture.left.getNextEventBatch();
		}
		if (!rightEvents.has_value()) {
			rightEvents = capture.right.getNextEventBatch();
		}

		// Feed the data into the slicer and reset the buffer
		if (leftEvents && rightEvents) {
			slicer.accept(*leftEvents, *rightEvents);
			leftEvents  = std::nullopt;
			rightEvents = std::nullopt;
		}

		// Wait for a small amount of time to avoid CPU overhaul
		cv::waitKey(1);
	}

	return 0;
}

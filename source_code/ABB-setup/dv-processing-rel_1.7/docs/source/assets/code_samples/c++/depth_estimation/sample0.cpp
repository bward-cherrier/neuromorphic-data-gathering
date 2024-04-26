#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/depth/semi_dense_stereo_matcher.hpp>
#include <dv-processing/io/stereo_capture.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>

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

	// Initialize a stereo block matcher with a stereo geometry from calibration and the preconfigured SGBM instance
	dv::SemiDenseStereoMatcher blockMatcher(std::make_unique<dv::camera::StereoGeometry>(leftCamera, rightCamera));

	// Initialization of a stereo event sliver
	dv::StereoEventStreamSlicer slicer;

	// Initialize a window to show previews of the output
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Local event buffers to implement overlapping window of events for accumulation
	dv::EventStore leftEventBuffer, rightEventBuffer;

	// Use one third of the resolution as count of events per accumulated frame
	const size_t eventCount = static_cast<size_t>(leftCamera.resolution.area()) / 3;

	// Register a callback to be done at 30Hz
	slicer.doEveryTimeInterval(33ms, [&blockMatcher, &leftEventBuffer, &rightEventBuffer, eventCount](
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

		// Pass these events into block matcher and estimate disparity, the matcher will accumulate frames
		// internally. The disparity output is 16-bit integer, that has sub-pixel precision.
		const auto disparity = blockMatcher.computeDisparity(leftEventBuffer, rightEventBuffer);

		// Convert disparity into 8-bit integers with scaling and normalize the output for a nice preview.
		// This loses the actual numeric value of the disparity, but it's a nice way to visualize the disparity.
		cv::Mat disparityU8, disparityColored;
		disparity.convertTo(disparityU8, CV_8UC1, 1.0 / 16.0);
		cv::normalize(disparityU8, disparityU8, 0, 255, cv::NORM_MINMAX);

		// Convert the accumulated frames into colored images for preview.
		std::vector<cv::Mat> images(3);
		cv::cvtColor(blockMatcher.getLeftFrame().image, images[0], cv::COLOR_GRAY2BGR);
		cv::cvtColor(blockMatcher.getRightFrame().image, images[1], cv::COLOR_GRAY2BGR);

		// Apply color-mapping to the disparity image, this will encode depth with color: red - close; blue - far.
		cv::applyColorMap(disparityU8, images[2], cv::COLORMAP_JET);

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

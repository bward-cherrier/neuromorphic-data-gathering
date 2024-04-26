#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/depth/semi_dense_stereo_matcher.hpp>
#include <dv-processing/features/event_feature_lk_tracker.hpp>
#include <dv-processing/io/stereo_capture.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>

#include <CLI/CLI.hpp>
#include <opencv2/highgui.hpp>

#include <csignal>
#include <thread>

static std::atomic<bool> globalShutdown(false);

static void handleShutdown(int) {
	globalShutdown.store(true);
}

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	// Variable to store the calibration path
	std::string calibrationPath;

	// Let's use CLI11 to handle command line arguments
	CLI::App app{"Depth estimation demo for a calibrated stereo camera with live preview"};
	app.add_option("-c,--calibration", calibrationPath, "Path to calibration file");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Install signal handlers for a clean shutdown
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	// Load the stereo calibration
	dv::camera::CalibrationSet calibration = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);

	const auto leftCam  = calibration.getCameraCalibration("C0");
	const auto rightCam = calibration.getCameraCalibration("C1");
	if (!leftCam.has_value() || !rightCam.has_value()) {
		throw dv::exceptions::RuntimeError("Calibration file does not contain a valid stereo camera calibration, "
										   "please check the calibration file.");
	}

	const std::string leftName  = leftCam->name;
	const std::string rightName = rightCam->name;

	std::cout << "Calibration contains cameras [L: " << leftName << "; R: " << rightName << "]" << std::endl;

	// Live stereo camera capture, handles time synchronization of the cameras
	dv::io::StereoCapture cameras(leftName, rightName);

	// Create a configure a SGBM instance with hand tuned paramters:
	// * minDisparity = 0, reasonable default
	// * numDisparities = 48, increases the range of measured disparities
	// * blockSize = 11, larger block sizes work better with reconstructed images, since the input is an edge map, there
	//   is too little texture for any fine-grained disparity calculations
	// * P1 and P2 are just constants calculated using recommendation: P1 = 8 * blockSize ^ 2;
	//   and P2 = 32 * blockSize ^ 2;
	// * disp12MaxDiff = 0, enables disparity cross-checking with right-left, allows no difference between the results,
	//   a good option for simple speckle filtering.
	// * preFilterCap = PREFILTER_NORMALIZED_RESPONSE, disables default sobel filtering on the input image for
	//   intermediate calculations, this can be done since accumulated images already contains edge maps and sobel
	//   filtering is redundant.
	// * uniquenessRatio = 15, some non-zero value to enable additional filtering on the output.
	// * speckleWindowSize = 240, a large speckle filter size value, the lack of texture creates a lot of speckles,
	//   so this enables aggressive speckle filtering.
	// * speckleRange = 1, also an aggressive filtering value.
	auto sgbm = cv::StereoSGBM::create(
		0, 48, 11, 8 * 11 * 11, 32 * 11 * 11, 0, cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE, 15, 240, 1);

	// We initialize a stereo geometry class
	auto geometry = std::make_unique<dv::camera::StereoGeometry>(*leftCam, *rightCam);

	// Instantiate the matcher instance based on calibration
	dv::SemiDenseStereoMatcher matcher(std::move(geometry), std::static_pointer_cast<cv::StereoMatcher>(sgbm));

	// Instantiate preview windows
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	cv::namedWindow("Disparity", cv::WINDOW_NORMAL);

	// Create a stereo event stream slicer to synchronize event streams
	dv::StereoEventStreamSlicer slicer;

	// Instantiate a background activity noise filter
	dv::noise::BackgroundActivityNoiseFilter noiseFilter(leftCam->resolution);

	// Callback function, will be performed at 30 FPS
	slicer.doEveryTimeInterval(33ms, [&matcher, &noiseFilter](const auto &left, const auto &right) {
		// Various variable holders for previews
		cv::Mat disparityU8, disparityColored, preview, previewColored, depthPreview;

		// Compute disparity between left / right images
		cv::Mat disparity = matcher.computeDisparity(left, right);

		// Convert, normalize and color code the disparity map
		disparity.convertTo(disparityU8, CV_8UC1, 1.0 / 16.0);
		cv::normalize(disparityU8, disparityU8, 0, 255, cv::NORM_MINMAX);
		cv::applyColorMap(disparityU8, disparityColored, cv::COLORMAP_JET);

		// Apply noise filtering to the left events, we want to calculate depth only on the
		// events that are most probable to be actual signal and not noise, since depth estimation
		// is an expensive procedure
		noiseFilter.accept(left);
		dv::EventStore locations = noiseFilter.generateEvents();

		// Perform the actual depth estimation of noise-filtered events
		dv::DepthEventStore depthEvents = matcher.estimateDepth(disparity, locations);

		// Sparse map will hold normalized depth values of the estimation
		cv::Mat sparseMap(disparity.size(), CV_8UC1, cv::Scalar(0));
		// Iterate over estimated depth events
		for (const dv::DepthEvent &depthEvent : depthEvents) {
			// Let's clamp the depth within 1.0 and 10.0 meters and normalize to 0-255
			double depthRange = (1.0 - std::clamp<double>((depthEvent.depth() - 1'000) / 10'000.0, 0., 1.)) * 255.;
			// Assign the normalized depth value to the pixel location
			sparseMap.at<uint8_t>(depthEvent.y(), depthEvent.x()) = static_cast<uint8_t>(depthRange);
		}
		// Apply color mapping to the sparse depth image to get a nice depth preview with fixed color coded depth
		cv::applyColorMap(sparseMap, depthPreview, cv::COLORMAP_JET);

		// Take out the accumulated left and right images from and concatenate them for preview
		cv::hconcat(matcher.getLeftFrame().image, matcher.getRightFrame().image, preview);
		cv::imshow("Preview", preview);

		// Concatenate the color coded disparity and depth for preview as well
		cv::hconcat(disparityColored, depthPreview, previewColored);
		cv::imshow("Disparity", previewColored);

		// If user presses ESC key (code 27), exit the application
		if (cv::waitKey(2) == 27) {
			globalShutdown = true;
		}
	});

	// This is the loop that handles data reading from camera
	std::optional<dv::EventStore> leftEvents = std::nullopt, rightEvents = std::nullopt;
	while ((cameras.left.isConnected() && cameras.right.isConnected()) || !globalShutdown) {
		// Read events from respective left right cameras
		if (!leftEvents.has_value()) {
			leftEvents = cameras.left.getNextEventBatch();
		}
		if (!rightEvents.has_value()) {
			rightEvents = cameras.right.getNextEventBatch();
		}
		// Feed the data into the slicer and reset the buffer
		if (leftEvents && rightEvents) {
			slicer.accept(*leftEvents, *rightEvents);
			leftEvents  = std::nullopt;
			rightEvents = std::nullopt;
		}
		else {
			// Wait for a small amount of time to avoid CPU overhaul
			usleep(1'000);
		}
	}

	return 0;
}

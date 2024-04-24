#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/cluster/mean_shift/event_store_adaptor.hpp>
#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/data/utilities.hpp>
#include <dv-processing/depth/sparse_event_block_matcher.hpp>
#include <dv-processing/features/keypoint_resampler.hpp>
#include <dv-processing/io/stereo_camera_recording.hpp>
#include <dv-processing/io/stereo_capture.hpp>
#include <dv-processing/visualization/colors.hpp>

#include <CLI/CLI.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>

static std::atomic<bool> globalShutdown(false);

static void handleShutdown(int) {
	globalShutdown.store(true);
}

void stereoDisplay(const std::string &name, const cv::Mat &left, const cv::Mat &right);

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	std::string path;
	std::string calibrationPath;
	std::string leftCameraName;
	std::string rightCameraName;

	CLI::App app{"Disparity estimation test"};

	app.add_option("-d,--data", path, "Dataset path [aedat4]")->check(CLI::ExistingFile);
	app.add_option("-c,--calibration", calibrationPath, "Path to calibration file")->check(CLI::ExistingFile);
	app.add_option("-l,--left-name", leftCameraName, "Left camera name")->required();
	app.add_option("-r,--right-name", rightCameraName, "Right camera name")->required();
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Aliases for left right readers to reduce code verbosity
	dv::io::CameraInputBase *left                            = nullptr;
	dv::io::CameraInputBase *right                           = nullptr;
	std::unique_ptr<dv::io::StereoCapture> capture           = nullptr;
	std::unique_ptr<dv::io::StereoCameraRecording> recording = nullptr;

	if (path.empty()) {
		capture = std::make_unique<dv::io::StereoCapture>(leftCameraName, rightCameraName);
		left    = &capture->left;
		right   = &capture->right;
	}
	else {
		recording = std::make_unique<dv::io::StereoCameraRecording>(path, leftCameraName, rightCameraName);
		left      = &recording->getLeftReader();
		right     = &recording->getRightReader();
	}

	// Basic configuration options here

	// Matching window size for the block matcher
	const cv::Size window(24, 24);
	// Minimum disparity value to measure
	const int minDisparity = 0;
	// Maximum disparity value
	const int maxDisparity = 40;
	// Minimum z-score value that a valid match can have
	const float minScore = 0.0f;

	// Initializing the matcher
	std::unique_ptr<dv::SparseEventBlockMatcher> matcher = nullptr;

	// Validate that we have same resolution.
	if (left->getEventResolution().value() != right->getEventResolution().value()) {
		throw dv::exceptions::RuntimeError("Sparse block matching requires stereo event inputs with same resolution.");
	}

	// Matcher can be used with and without calibration. Without calibration is useful if rectification is already
	// performed on the data, for this sample it is only useful for demonstration purposes.
	if (calibrationPath.empty()) {
		// Initialize the matcher without rectification, uses only the resolution
		matcher = std::make_unique<dv::SparseEventBlockMatcher>(
			left->getEventResolution().value(), window, maxDisparity, minDisparity, minScore);
	}
	else {
		// Calibration is provided, read the calibration file
		const auto calibration = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
		// Constructing the stereo geometry instance from calibration
		auto stereoGeometry = std::make_unique<dv::camera::StereoGeometry>(
			calibration.getCameraCalibrationByName(leftCameraName).value(),
			calibration.getCameraCalibrationByName(rightCameraName).value());
		// Initialize the block matcher with rectification
		matcher = std::make_unique<dv::SparseEventBlockMatcher>(
			std::move(stereoGeometry), window, maxDisparity, minDisparity, minScore);
	}

	// Create a stereo event stream slicer to synchronize event streams
	dv::StereoEventStreamSlicer slicer;

	// Preview window, visualization only
	cv::namedWindow("preview", cv::WINDOW_NORMAL);

	// This sample will run in three threads:
	// - clustering thread: runs mean shift clustering on input data
	// - disparity thread: performs disparity estimation on cluster centers
	// - main thread: data read from camera and visualization from disparity thread

	// Here we declare lockfree queues for inter-thread communication
	boost::lockfree::spsc_queue<std::tuple<dv::EventStore, dv::EventStore>> clusteringQueue(100);
	boost::lockfree::spsc_queue<std::tuple<dv::EventStore, dv::EventStore,
		std::vector<dv::TimedKeyPoint, Eigen::aligned_allocator<dv::TimedKeyPoint>>>>
		disparityQueue(100);
	boost::lockfree::spsc_queue<std::tuple<cv::Mat, cv::Mat, std::vector<dv::SparseEventBlockMatcher::PixelDisparity>>>
		visualizationQueue(100);

	// Clustering thread
	std::thread clusteringThread([&] {
		while (!globalShutdown) {
			// Read any input data from clustering queue
			clusteringQueue.consume_all([&disparityQueue](const std::tuple<dv::EventStore, dv::EventStore> &input) {
				// Unpack input tuple
				const auto &[leftEvents, rightEvents] = input;
				// Number of clusters to extract
				constexpr int numClusters = 100;

				// Initialize the mean-shift clustering algorithm
				dv::cluster::mean_shift::MeanShiftEventStoreAdaptor meanShift(
					leftEvents.sliceTime(leftEvents.getHighestTime() - 500), 20.f, 1.0f, 5, numClusters);

				// Extract centers
				auto centers = meanShift.findClusterCentres<dv::cluster::mean_shift::kernel::Epanechnikov>();

				// Pass output into disparity queue
				disparityQueue.push(std::make_tuple(leftEvents, rightEvents, centers));
			});
			std::this_thread::sleep_for(100us);
		}
	});

	// Disparity estimation thread
	std::thread disparityThread([&] {
		while (!globalShutdown) {
			// Read any input data coming into the disparity queue
			disparityQueue.consume_all(
				[&matcher, &visualizationQueue](const std::tuple<dv::EventStore, dv::EventStore,
					std::vector<dv::TimedKeyPoint, Eigen::aligned_allocator<dv::TimedKeyPoint>>> &input) {
					// Unpack input tuple
					const auto &[leftEvents, rightEvents, centers] = input;

					// Run the disparity estimation
					const std::vector<dv::SparseEventBlockMatcher::PixelDisparity> estimates
						= matcher->computeDisparitySparse(
							leftEvents, rightEvents, dv::data::convertToCvPoints(centers));

					// Visualization: convert the input accumulated images into colored images for preview
					cv::Mat leftRGB, rightRGB;
					cv::cvtColor(matcher->getLeftFrame().image, leftRGB, cv::COLOR_GRAY2BGR);
					cv::cvtColor(matcher->getRightFrame().image, rightRGB, cv::COLOR_GRAY2BGR);

					// Pass the images and disparity estimations into visualization queue
					visualizationQueue.push(std::make_tuple(leftRGB, rightRGB, estimates));
				});
			std::this_thread::sleep_for(100us);
		}
	});

	// Callback function, will be performed at 100 FPS
	slicer.doEveryTimeInterval(10ms, [&clusteringQueue, &visualizationQueue, &window](
										 const auto &left, const auto &right) {
		// We will search for 100 points of interests (clusters)
		clusteringQueue.push(std::make_tuple(left, right));

		// The following lines takes the accumulated images from the algorithm and converts them into color images
		// for visualization
		visualizationQueue.consume_all(
			[&window](
				const std::tuple<cv::Mat, cv::Mat, std::vector<dv::SparseEventBlockMatcher::PixelDisparity>> &input) {
				const auto &[leftRGB, rightRGB, estimates] = input;
				// Iterate through the results, we want to remove cluster centers which were not successfully estimated
				// in the block matching.
				int32_t index = 0;
				for (const auto &point : estimates) {
					// If point estimation is invalid, remove it from the clustering
					if (!point.valid) {
						// Removing from centers list.
						continue;
					}

					// The rest of the code performs drawing of the match according to the disparity value on the
					// preview images.
					cv::Scalar color = dv::visualization::colors::someNeonColor(index++);
					// Draw some nice colored markers and rectangles.
					cv::drawMarker(rightRGB, *point.matchedPosition, color, cv::MARKER_CROSS, 7);
					cv::rectangle(rightRGB,
						cv::Rect(point.matchedPosition->x - (window.width / 2),
							point.matchedPosition->y - (window.height / 2), window.width, window.height),
						color);
					cv::rectangle(leftRGB,
						cv::Rect(point.templatePosition->x - (window.width / 2),
							point.templatePosition->y - (window.height / 2), window.width, window.height),
						color);
				}

				// This function shows the preview window.
				stereoDisplay("preview", leftRGB, rightRGB);
			}

		);
	});

	// Continuously read data from the live camera
	while (!globalShutdown && left->isRunning() && right->isRunning()) {
		slicer.accept(left->getNextEventBatch(), right->getNextEventBatch());
		std::this_thread::sleep_for(1ms);
	}

	globalShutdown = true;

	// Close and exit
	clusteringThread.join();
	disparityThread.join();

	return EXIT_SUCCESS;
}

void stereoDisplay(const std::string &name, const cv::Mat &left, const cv::Mat &right) {
	// Nice preview of a stereo pair of images
	cv::Mat preview;
	cv::putText(left, "Left", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 2.0, dv::visualization::colors::silver);
	cv::putText(right, "Right", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 2.0, dv::visualization::colors::silver);
	cv::rectangle(left, cv::Point(0, 0), cv::Point(640, 480), dv::visualization::colors::silver);
	cv::rectangle(right, cv::Point(0, 0), cv::Point(640, 480), dv::visualization::colors::silver);
	cv::hconcat(left, right, preview);
	cv::imshow(name, preview);
	// Pause if 'p' key is pressed
	if (cv::waitKey(2) == 'p') {
		cv::waitKey();
	}
}

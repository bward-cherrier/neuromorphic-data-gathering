#include <dv-processing/features/event_blob_detector.hpp>
#include <dv-processing/features/feature_detector.hpp>
#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/colors.hpp>

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Open any camera
	dv::io::CameraCapture capture;

	// Make sure it supports event stream output, throw an error otherwise
	if (!capture.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Input camera does not provide an event stream.");
	}

	const auto resolution = *capture.getEventResolution();

	// Initialize an accumulator with camera resolution (for visualization only)
	dv::Accumulator accumulator(resolution);

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Let's detect up to 100 features
	const size_t numberOfFeatures = 100;

	// define preprocessing step (here we do nothing but show how it can be implemented)
	std::function<void(cv::Mat &)> preprocess = [](cv::Mat &image) {
	};

	// Define number of pyr-down to be applied to the image before detecting blobs. If pyramidLevel is set to zero, the
	// blobs will be detected on the original image resolution, if pyramid level is one, the image will be down sampled
	// by a factor of 2 (factor of 4 if pyramid level is 2 and so on...) before performing the detection. The blobs will
	// then be scaled back to the original resolution size.
	const int32_t pyramidLevel = 0;
	// Create an event-based blob detector
	const auto blobDetector = std::make_shared<dv::features::EventBlobDetector>(resolution, pyramidLevel, preprocess);
	auto detector           = dv::features::EventFeatureBlobDetector(resolution, blobDetector);

	// Initialize a slicer
	dv::EventStreamSlicer slicer;

	// Register a callback every 33 milliseconds
	slicer.doEveryTimeInterval(33ms, [&accumulator, &detector](const dv::EventStore &events) {
		// Run the feature detection on the incoming events.
		const auto features = detector.runDetection(events, numberOfFeatures);

		// Run accumulator for an image, it is going to be used only for preview
		accumulator.accept(events);
		const dv::Frame frame = accumulator.generateFrame();

		// Create a colored preview image by converting from grayscale to BGR
		cv::Mat preview;
		cv::cvtColor(frame.image, preview, cv::COLOR_GRAY2BGR);

		// Draw detected features
		cv::drawKeypoints(preview, dv::data::fromTimedKeyPoints(features), preview);

		// Show the accumulated image
		cv::imshow("Preview", preview);
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

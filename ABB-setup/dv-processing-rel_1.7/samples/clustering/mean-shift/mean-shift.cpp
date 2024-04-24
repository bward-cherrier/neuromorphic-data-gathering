/**
 * Visualize how event clustering works.
 *
 * This sample requires an event camera and a remote controller. Run the code and point a remote controller at the
 * camera. When you press any button the transmitter LED of the controller will start sending a light signal which
 * then will be captured by our camera and result in increased event activity. Our clustering algorithm detects areas
 * of increased activity and marks them on the preview image.
 */

#include <dv-processing/cluster/mean_shift.hpp>
#include <dv-processing/core/frame.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include <opencv2/highgui.hpp>

using namespace dv::cluster::mean_shift;

int main(int ac, char **av) {
	// Capture used to communicate with the camera
	dv::io::CameraCapture capture;

	if (!capture.getEventResolution().has_value()) {
		throw dv::exceptions::RuntimeError{"Camera has no event stream."};
	}

	// Accumulator generating a preview image
	dv::Accumulator accumulator(*capture.getEventResolution());

	// Vector of lat detected clusters of events, used to initialize at the next iteration
	MeanShiftEventStoreAdaptor::VectorOfVectors lastDetectedClusters;

	bool shutdown = false;
	while (!shutdown && capture.isConnected()) {
		// Get the new batch of events if available
		auto events = capture.getNextEventBatch();
		if (!events.has_value()) {
			continue;
		}
		auto store = events.value();

		// Generate a preview image
		accumulator.accept(store);
		cv::Mat preview;
		cv::cvtColor(accumulator.generateFrame().image, preview, cv::COLOR_GRAY2BGR);

		// Perform a step of mean shift algorithm to detect event clusters
		auto startingPoints = MeanShiftEventStoreAdaptor::generateStartingPointsFromData(10, store);
		startingPoints.insert(startingPoints.end(), lastDetectedClusters.begin(), lastDetectedClusters.end());
		lastDetectedClusters.clear();
		MeanShiftEventStoreAdaptor meanShift(store, 15.f, 0.001f, 1000, startingPoints);
		const auto [centers, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		// Visualize the detected clusters
		for (size_t i = 0; i < centers.size(); ++i) {
			const auto c       = centers[i];
			const auto nEvents = counts[i];

			if (nEvents < 2500) {
				// We expect the remote controller to produce at least 2500 events per events batch
				continue;
			}

			// Use the current cluster as starting point at the next iteration
			lastDetectedClusters.emplace_back(dv::TimedKeyPoint(c.pt, 0.f, 0.f, 0.f, 0, 0, 0));

			const auto x = static_cast<int>(c.pt.x());
			const auto y = static_cast<int>(c.pt.y());
			cv::circle(preview, cv::Point2i(x, y), 5, {0, 255, 0}, cv::FILLED);
		}

		cv::imshow("Clustering demo", preview);
		if (cv::waitKey(1) == 'q') {
			shutdown = true;
		}
	}

	return EXIT_SUCCESS;
}

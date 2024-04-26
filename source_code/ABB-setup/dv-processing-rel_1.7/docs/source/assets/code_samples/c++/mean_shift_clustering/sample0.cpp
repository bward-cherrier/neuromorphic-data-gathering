#include <dv-processing/cluster/mean_shift/event_store_adaptor.hpp>
#include <dv-processing/core/event.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/visualization/events_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Use VGA resolution
	const cv::Size resolution(640, 480);

	// Initialize a list of clusters for synthetic data generation
	const std::vector<dv::Point2f> clusters(
		{dv::Point2f(100.f, 100.f), dv::Point2f(462.f, 25.f), dv::Point2f(105.f, 340.f), dv::Point2f(540.f, 420.f)});

	// Generate some random events for a background
	dv::EventStore events = dv::data::generate::uniformlyDistributedEvents(0, resolution, 10'000);

	// Declare a region filter which we will use to filter out-of-bounds events in the next step
	dv::EventRegionFilter filter(cv::Rect(0, 0, resolution.width, resolution.height));

	for (const auto &cluster : clusters) {
		// Generate a batch of normally distributed events around each of the cluster centers
		filter.accept(dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(15.f, 15.f), 5'000));

		// Apply region filter to the events to filter out events outside valid dimensions
		events += filter.generateEvents();
	}

	// Initialize mean shift clustering algorithm, with initial parameters of:
	// bandwidth = 100, this is pixel search radius around a point
	// conv = 0.01, the search converges when the magnitude of mean-shift vector is below this value
	// maxIter = 10000, maximum number of mean-shift update iterations
	// numStartingPoints = 100, number of randomly selected starting points
	dv::cluster::mean_shift::MeanShiftEventStoreAdaptor meanShift(events, 100.0f, 0.01f, 10000, 100);

	// Perform the mean-shift, the algorithm returns a tuple of center coordinates, labels, count of event in
	// the cluster, and variances of the cluster
	auto [centers, labels, counts, variances] = meanShift.fit();

	// Let's assign the cluster size to the response value of the center keypoint
	for (int i = 0; i < centers.size(); i++) {
		centers[i].response = static_cast<float>(counts[i]);
	}

	// Sort the estimated centers by the number of events in cluster, the values are sorted in descending order
	std::sort(centers.begin(), centers.end(), [](const auto &a, const auto &b) {
		return a.response > b.response;
	});

	// Choose top four center with most events; these centers should be close to initial hardcoded cluster centers
	if (centers.size() > 4) {
		centers.resize(4);
	}

	// Use event visualizer to generate a preview image
	dv::visualization::EventVisualizer visualizer(resolution);
	auto preview = visualizer.generateImage(events);

	// Draw markers on each of the center coordinates
	for (const auto &center : centers) {
		cv::drawMarker(preview, cv::Point2f(center.pt.x(), center.pt.y()), dv::visualization::colors::red);
	}

	// Show the preview image with detected cluster centers
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	cv::imshow("Preview", preview);
	cv::waitKey();

	return 0;
}

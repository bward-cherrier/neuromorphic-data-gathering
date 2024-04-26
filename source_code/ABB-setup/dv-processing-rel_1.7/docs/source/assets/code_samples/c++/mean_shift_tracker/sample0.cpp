#include <dv-processing/core/event.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/features/mean_shift_tracker.hpp>
#include <dv-processing/visualization/events_visualizer.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

[[nodiscard]] dv::EventStore generateEventClustersAtTime(const int64_t time, const std::vector<dv::Point2f> &clusters,
	const uint64_t numIter, const cv::Size &resolution, const int shift = -5);

int main() {
	using namespace std::chrono_literals;

	// Use VGA resolution
	const cv::Size resolution(640, 480);

	// Initialize a slicer
	dv::EventStreamSlicer slicer;

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Initialize a list of clusters for synthetic data generation
	const std::vector<dv::Point2f> clusters(
		{dv::Point2f(550.f, 400.f), dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f)});

	// Generate some random events for a background
	dv::EventStore events = dv::data::generate::uniformlyDistributedEvents(0, resolution, 10'000);

	std::vector<int64_t> timestamps = {0, 40000, 80000, 120000, 160000, 200000, 240000, 280000, 320000, 360000};

	uint64_t numIter = 0;
	for (const auto time : timestamps) {
		auto eventCluster = generateEventClustersAtTime(time, clusters, numIter, resolution);
		events            += eventCluster;
		events            += dv::data::generate::uniformlyDistributedEvents(time, resolution, 10000, numIter);
		numIter++;
	}

	// Bandwidth value defining the size of the search window in which updated track location will be searched
	const int bandwidth = 10;

	// Time window used for the normalized time surface computation. In this case we take the last 50ms of events and
	// compute a normalized time surface over them
	const dv::Duration timeWindow = 50ms;

	// Initialize a mean shift tracker.
	dv::features::MeanShiftTracker meanShift = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

	dv::visualization::EventVisualizer visualizer(resolution);

	// Register a callback every 40 milliseconds
	slicer.doEveryTimeInterval(40ms, [&](const dv::EventStore &events) {
		meanShift.accept(events);
		auto meanShiftTracks = meanShift.runTracking();

		if (!meanShiftTracks) {
			return;
		}

		// visualize mean shift tracks
		auto preview = visualizer.generateImage(events);
		auto points  = dv::data::fromTimedKeyPoints(meanShiftTracks->keypoints);
		cv::drawKeypoints(preview, points, preview, dv::visualization::colors::red);

		cv::imshow("Preview", preview);
		cv::waitKey(300);
	});

	slicer.accept(events);

	return EXIT_SUCCESS;
}

dv::EventStore generateEventClustersAtTime(const int64_t time, const std::vector<dv::Point2f> &clusters,
	const uint64_t numIter, const cv::Size &resolution, const int shift) {
	// Declare a region filter which we will use to filter out-of-bounds events in the next step
	dv::EventRegionFilter filter(cv::Rect(0, 0, resolution.width, resolution.height));
	const float offset = static_cast<float>(shift * static_cast<int>(numIter));
	dv::EventStore eventFiltered;
	for (const auto &cluster : clusters) {
		const auto xShift       = cluster.x() + offset;
		const auto yShift       = cluster.y() + offset;
		const dv::Point2f point = dv::Point2f(xShift, yShift);
		// Generate a batch of normally distributed events around each of the cluster centers
		filter.accept(dv::data::generate::normallyDistributedEvents(time, point, dv::Point2f(3.f, 3.f), 1'000));

		// Apply region filter to the events to filter out events outside valid dimensions
		eventFiltered += filter.generateEvents();
	}

	return eventFiltered;
}

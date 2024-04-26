#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/features/event_blob_detector.hpp"
#include "../../include/dv-processing/features/feature_detector.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"no_events"_test = [] {
		const cv::Size resolution(640, 480);
		auto blobDetector     = dv::features::EventBlobDetector(resolution);
		dv::EventStore events = dv::EventStore();
		auto blobs            = blobDetector.detect(events);

		// check blob detector didn't detect any cluster
		expect(eq(blobs.size(), 0));
	};

	"detection"_test = [] {
		const cv::Size resolution(640, 480);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f), dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 3.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 1'000);
		}

		auto blobDetector = dv::features::EventBlobDetector(resolution);
		auto blobs        = blobDetector.detect(events);

		// sort blobs in ascending x coordinates
		std::sort(blobs.begin(), blobs.end(), [](const auto &a, const auto &b) {
			return a.pt.x() < b.pt.x();
		});

		// check blob detector detects all created cluster
		expect(eq(blobs.size(), 3));

		// check detected blob center not further than 1.5 * std. Event data are generated from uniform distribution.
		// ~68% of the event are generated inside 1 std. It is reasonable to expect that the center is detected inside
		// the one standard deviation distance. For safety we enforce it to be not further than 1.5 std away.
		expect(le(abs(blobs.at(0).pt.x() - clusters[0].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[0].y()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.x() - clusters[1].x()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.y() - clusters[1].y()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.x() - clusters[2].x()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.y() - clusters[2].y()), 1.5 * std));

		// update mask s.t. discards 1st blob in cluster list
		cv::Rect roiToZero(0, 0, 200, 400);
		cv::Rect roi(0, 0, resolution.width, resolution.height);
		cv::Mat mask    = cv::Mat::ones(resolution, CV_8UC1) * 255;
		mask(roiToZero) = 0;
		blobs           = blobDetector.detect(events, roi, mask);

		// sort blobs in ascending x coordinates
		std::sort(blobs.begin(), blobs.end(), [](const auto &a, const auto &b) {
			return a.pt.x() < b.pt.x();
		});

		// check blob detector detects only blobs belonging to mask
		expect(eq(blobs.size(), 2));

		expect(le(abs(blobs.at(0).pt.x() - clusters[1].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[1].y()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.x() - clusters[2].x()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.y() - clusters[2].y()), 1.5 * std));

		// set roi s.t. only last blob is detected
		cv::Rect roiToKeep(500, 350, 100, 100);
		blobs = blobDetector.detect(events, roiToKeep, mask);

		// check blob detector detects only blobs belonging to mask
		expect(eq(blobs.size(), 1));

		expect(le(abs(blobs.at(0).pt.x() - clusters[2].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[2].y()), 1.5 * std));
	};

	"apply_pyr_down_x2"_test = [] {
		const cv::Size resolution(640, 480);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f), dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 3.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 1'000);
		}

		int pyramidLevel  = 2;
		auto blobDetector = dv::features::EventBlobDetector(resolution, pyramidLevel);
		cv::Rect roi(0, 0, resolution.width, resolution.height);
		cv::Mat mask = cv::Mat::ones(resolution, CV_8UC1) * 255;

		auto blobs = blobDetector.detect(events, roi, mask);

		// sort blobs in ascending x coordinates
		std::sort(blobs.begin(), blobs.end(), [](const auto &a, const auto &b) {
			return a.pt.x() < b.pt.x();
		});

		// check blob detector detects all created cluster
		expect(eq(blobs.size(), 3));

		// check detected blob center not further than 1.5 * std. Event data are generated from uniform distribution.
		// ~68% of the event are generated inside 1 std. It is reasonable to expect that the center is detected inside
		// the one standard deviation distance. For safety we enforce it to be not further than 1.5 std away.
		expect(le(abs(blobs.at(0).pt.x() - clusters[0].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[0].y()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.x() - clusters[1].x()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.y() - clusters[1].y()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.x() - clusters[2].x()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.y() - clusters[2].y()), 1.5 * std));
	};

	"apply_pyr_down_x3"_test = [] {
		const cv::Size resolution(640, 480);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f), dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 9.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 3'000);
		}

		int pyramidLevel  = 3;
		auto blobDetector = dv::features::EventBlobDetector(resolution, pyramidLevel);
		cv::Rect roi(0, 0, resolution.width, resolution.height);
		cv::Mat mask = cv::Mat::ones(resolution, CV_8UC1) * 255;

		auto blobs = blobDetector.detect(events, roi, mask);

		// sort blobs in ascending x coordinates
		std::sort(blobs.begin(), blobs.end(), [](const auto &a, const auto &b) {
			return a.pt.x() < b.pt.x();
		});

		// check blob detector detects all created cluster
		expect(eq(blobs.size(), 3));

		// check detected blob center not further than 1.5 * std. Event data are generated from uniform distribution.
		// ~68% of the event are generated inside 1 std. It is reasonable to expect that the center is detected inside
		// the one standard deviation distance. For safety we enforce it to be not further than 1.5 std away.
		expect(le(abs(blobs.at(0).pt.x() - clusters[0].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[0].y()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.x() - clusters[1].x()), 1.5 * std));
		expect(le(abs(blobs.at(1).pt.y() - clusters[1].y()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.x() - clusters[2].x()), 1.5 * std));
		expect(le(abs(blobs.at(2).pt.y() - clusters[2].y()), 1.5 * std));
	};

	"apply_pyr_down_and_roi"_test = [] {
		const cv::Size resolution(640, 480);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f), dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 9.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 3'000);
		}

		int pyramidLevel  = 3;
		auto blobDetector = dv::features::EventBlobDetector(resolution, pyramidLevel);
		cv::Rect roi(0, 0, 100, resolution.height);
		cv::Mat mask = cv::Mat::ones(resolution, CV_8UC1) * 255;

		auto blobs = blobDetector.detect(events, roi, mask);

		// sort blobs in ascending x coordinates
		std::sort(blobs.begin(), blobs.end(), [](const auto &a, const auto &b) {
			return a.pt.x() < b.pt.x();
		});

		// check blob detector detects all created cluster
		expect(eq(blobs.size(), 1));

		// check detected blob center not further than 1.5 * std. Event data are generated from uniform distribution.
		// ~68% of the event are generated inside 1 std. It is reasonable to expect that the center is detected inside
		// the one standard deviation distance. For safety we enforce it to be not further than 1.5 std away.
		expect(le(abs(blobs.at(0).pt.x() - clusters[0].x()), 1.5 * std));
		expect(le(abs(blobs.at(0).pt.y() - clusters[0].y()), 1.5 * std));
	};

	"preprocess_fcn_set_image_to_zero"_test = [] {
		std::function<void(cv::Mat &)> preprocess = [](cv::Mat &image) {
			image = 0;
		};

		const cv::Size resolution(640, 480);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f), dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 9.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 3'000);
		}

		const int pyramidLevel          = 3;
		auto blobDetectorWithPreprocess = dv::features::EventBlobDetector(resolution, pyramidLevel, preprocess);
		auto blobDetectorNoPreprocess   = dv::features::EventBlobDetector(resolution, pyramidLevel);
		cv::Rect roi(0, 0, resolution.width, resolution.height);
		cv::Mat mask = cv::Mat::ones(resolution, CV_8UC1) * 255;

		auto blobsWithPreprocess = blobDetectorWithPreprocess.detect(events, roi, mask);
		auto blobs               = blobDetectorNoPreprocess.detect(events, roi, mask);

		// check blob detector detects all created cluster
		expect(eq(blobsWithPreprocess.size(), 0));
		expect(eq(blobs.size(), 3));
	};

	"blob_finder_as_feature_detector_k"_test = [] {
		const cv::Size resolution(640, 480);
		const auto detector = std::make_shared<dv::features::EventBlobDetector>(resolution);
		auto blobDetector   = dv::features::EventFeatureBlobDetector(resolution, detector);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(550.f, 400.f), dv::Point2f(70.f, 300.f), dv::Point2f(305.f, 100.f)});

		dv::EventStore events;
		const float std = 3.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(0, cluster, dv::Point2f(std, std), 1'000);
		}

		const int maxNumPointsToDetect = 2;
		auto blobs                     = blobDetector.runDetection(events, maxNumPointsToDetect);

		expect(eq(blobs.size(), maxNumPointsToDetect));
	};

	return EXIT_SUCCESS;
}

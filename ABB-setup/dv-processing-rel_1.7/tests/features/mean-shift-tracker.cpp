#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/features/mean_shift_tracker.hpp"

#include "boost/ut.hpp"

#include <chrono>

int main() {
	using namespace boost::ut;

	"constructor_and_setters"_test = []() {
		const cv::Size resolution(100, 100);
		int bandwidth           = 10;
		dv::Duration timeWindow = std::chrono::milliseconds(10);
		auto meanShift          = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

		expect(eq(meanShift.getBandwidth(), bandwidth));
		expect(eq(meanShift.getTimeWindow().count(), timeWindow.count()));
		expect(eq(meanShift.getStepSize(), 0.5));

		meanShift.setBandwidth(15);
		expect(eq(meanShift.getBandwidth(), 15));

		meanShift.setStepSize(0.8f);
		expect(le(abs(meanShift.getStepSize() - 0.8f), 1e-6));

		dv::Duration newTimeWindow = std::chrono::microseconds(2);
		meanShift.setTimeWindow(newTimeWindow);
		expect(eq(meanShift.getTimeWindow().count(), newTimeWindow.count()));

		meanShift.setWeightMultiplier(2.5);
		expect(eq(meanShift.getWeightMultiplier(), 2.5));

		expect(throws([&] {
			// Throws an exception here
			meanShift.setStepSize(-1);
		}));

		expect(throws([&] {
			// Throws an exception here
			meanShift.setStepSize(3);
		}));

		expect(throws([&] {
			// Throws an exception here
			meanShift.setBandwidth(-1);
		}));
	};

	"no_data"_test = []() {
		const cv::Size resolution(100, 100);
		int bandwidth           = 10;
		dv::Duration timeWindow = std::chrono::milliseconds(10);
		auto meanShift          = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

		// run tracking without feeding event data
		auto outputTracks = meanShift.runTracking();
		// no event fed to time surface so output tracks should be the same as input
		expect(eq(outputTracks->keypoints.size(), 0));
		expect(eq(outputTracks->timestamp, 0));
	};

	"single_track"_test = []() {
		const cv::Size resolution(640, 480);

		int bandwidth           = 10;
		dv::Duration timeWindow = std::chrono::milliseconds(10);
		auto meanShift          = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters({dv::Point2f(550.f, 400.f)});

		dv::EventStore events;
		const float std = 3.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(10000000, cluster, dv::Point2f(std, std), 1'000);
		}

		meanShift.accept(events);
		auto tracks = meanShift.runTracking();

		expect(eq(tracks->keypoints.size(), 1));
		expect(eq(tracks->timestamp, 10000000));
		expect(eq(tracks->asKeyFrame, true));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(le(abs(tracks->keypoints[0].pt.x() - clusters[0].x()), std));
		expect(le(abs(tracks->keypoints[0].pt.y() - clusters[0].y()), std));
	};

	"multiple_tracks"_test = []() {
		const cv::Size resolution(640, 480);

		int bandwidth           = 20;
		dv::Duration timeWindow = std::chrono::milliseconds(100);
		auto meanShift          = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters({dv::Point2f(550.f, 400.f), dv::Point2f(70.f, 300.f)});
		std::vector<int64_t> timestamps = {10020000, 10080000, 10160000, 10240000, 10320000};

		dv::EventStore events;
		const float std = 3.f;
		for (const auto &cluster : clusters) {
			// Generate a batch of normally distributed events around each of the cluster centers
			events += dv::data::generate::normallyDistributedEvents(
				timestamps.front(), cluster, dv::Point2f(std, std), 1'000);
		}

		meanShift.accept(events);
		auto tracks = meanShift.runTracking();

		// test that two tracks have been detected
		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));
		expect(eq(tracks->asKeyFrame, true));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(eq(tracks->keypoints[1].class_id, 1));
		expect(le(abs(tracks->keypoints[0].pt.x() - clusters[0].x()), std));
		expect(le(abs(tracks->keypoints[0].pt.y() - clusters[0].y()), std));
		expect(le(abs(tracks->keypoints[1].pt.x() - clusters[1].x()), std));
		expect(le(abs(tracks->keypoints[1].pt.y() - clusters[1].y()), std));

		// test that the two tracks are moved to location defined by shift
		float shift = 4;
		events      = dv::EventStore();
		int numIter = 1;
		for (const auto time : timestamps) {
			for (const auto &cluster : clusters) {
				dv::Point2f point(cluster.x() + shift * static_cast<float>(numIter),
					cluster.y() + shift * static_cast<float>(numIter));
				// Generate a batch of normally distributed events around each of the cluster centers
				events += dv::data::generate::normallyDistributedEvents(time, point, dv::Point2f(std, std), 1'000);
			}
			numIter += 1;
		}

		meanShift.accept(events);
		tracks = meanShift.runTracking();

		// test that two tracks - shifted - have been tracked. Since the blobs have some given size, we don't expect to
		// land exactly where the blob is moved but somewhere close to that location. Here blobs are generated as
		// uniform distributions, and we expect not to be more than one standard deviation away from expected location
		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));
		expect(eq(tracks->asKeyFrame, false));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(eq(tracks->keypoints[1].class_id, 1));
		expect(le(
			abs(tracks->keypoints[0].pt.x() - (clusters[0].x() + shift * static_cast<float>(timestamps.size()))), std));
		expect(le(
			abs(tracks->keypoints[0].pt.y() - (clusters[0].y() + shift * static_cast<float>(timestamps.size()))), std));
		expect(le(
			abs(tracks->keypoints[1].pt.x() - (clusters[1].x() + shift * static_cast<float>(timestamps.size()))), std));
		expect(le(
			abs(tracks->keypoints[1].pt.y() - (clusters[1].y() + shift * static_cast<float>(timestamps.size()))), std));

		// update one of the two tracks only
		std::vector<int64_t> timestampsFirstTrack    = {};
		std::vector<int64_t> newTimestampsFirstTrack = {10360000, 10400000, 10440000, 10480000, 10520000};

		// shift track one while keeping track two at same location
		events  = dv::EventStore();
		numIter = 1;
		timestampsFirstTrack.insert(timestampsFirstTrack.end(), timestamps.begin(), timestamps.end());
		timestampsFirstTrack.insert(
			timestampsFirstTrack.end(), newTimestampsFirstTrack.begin(), newTimestampsFirstTrack.end());

		for (const auto time : timestampsFirstTrack) {
			dv::Point2f point(clusters[0].x() + shift * static_cast<float>(numIter),
				clusters[0].y() + shift * static_cast<float>(numIter));
			// Generate a batch of normally distributed events around each of the cluster centers
			events  += dv::data::generate::normallyDistributedEvents(time, point, dv::Point2f(std, std), 1'000);
			numIter += 1;
		}

		meanShift.accept(events);
		tracks = meanShift.runTracking();

		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));
		expect(eq(tracks->asKeyFrame, false));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(eq(tracks->keypoints[1].class_id, 1));
		expect(le(abs(tracks->keypoints[0].pt.x()
					  - (clusters[0].x() + shift * static_cast<float>(timestampsFirstTrack.size()))),
			std));
		expect(le(abs(tracks->keypoints[0].pt.y()
					  - (clusters[0].y() + shift * static_cast<float>(timestampsFirstTrack.size()))),
			std));
		expect(le(
			abs(tracks->keypoints[1].pt.x() - (clusters[1].x() + shift * static_cast<float>(timestamps.size()))), std));
		expect(le(
			abs(tracks->keypoints[1].pt.y() - (clusters[1].y() + shift * static_cast<float>(timestamps.size()))), std));
	};

	"multiple_tracks_same_window"_test = []() {
		// update bandwidth to full image plane and test that only first track is updated to location defined by
		// new set of events. Note that we update center based on the order of the inputTracks fed to the run function.
		// If a new candidate track fall inside the bandwidth area covered by a previously found new track, we do not
		// update this track and keep its input value.

		const cv::Size resolution(640, 480);
		dv::Duration timeWindow = std::chrono::milliseconds(100);

		std::vector<int64_t> newTimestampsAllTracks = {10360000, 10400000, 10440000, 10480000, 10520000};
		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters({dv::Point2f(550.f, 400.f), dv::Point2f(70.f, 300.f)});

		dv::EventStore events;
		const float std = 3.f;

		// Generate a batch of normally distributed events around each of the cluster centers
		events += dv::data::generate::normallyDistributedEvents(0, clusters[0], dv::Point2f(std, std), 1'000);
		events += dv::data::generate::normallyDistributedEvents(0, clusters[1], dv::Point2f(std, std), 1'000);

		const int bandwidth = 640;
		auto meanShift      = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);
		meanShift.accept(events);
		auto tracks = meanShift.runTracking();

		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, 0));
		expect(eq(tracks->asKeyFrame, true));

		float shift = 4;
		int numIter = 1;
		events      = dv::EventStore();

		for (const auto time : newTimestampsAllTracks) {
			for (const auto &cluster : clusters) {
				dv::Point2f point(cluster.x() + shift * static_cast<float>(numIter),
					cluster.y() + shift * static_cast<float>(numIter));
				// Generate a batch of normally distributed events around each of the cluster centers
				events += dv::data::generate::normallyDistributedEvents(time, point, dv::Point2f(std, std), 1'000);
			}
			numIter += 1;
		}

		meanShift.accept(events);
		tracks = meanShift.runTracking();

		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));
		expect(eq(tracks->asKeyFrame, false));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(eq(tracks->keypoints[1].class_id, 1));
		expect(le(abs(tracks->keypoints[0].pt.x()
					  - (clusters[0].x() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
		expect(le(abs(tracks->keypoints[0].pt.y()
					  - (clusters[0].y() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
		expect(le(abs(tracks->keypoints[1].pt.x() - clusters[1].x()), std));
		expect(le(abs(tracks->keypoints[1].pt.y() - clusters[1].y()), std));

		// set bandwidth to smaller window and check that 2nd track is updated as well
		int newBandwidth = 20;
		meanShift.setBandwidth(newBandwidth);
		tracks = meanShift.runTracking();

		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));
		expect(eq(tracks->asKeyFrame, false));
		expect(eq(tracks->keypoints[0].class_id, 0));
		expect(eq(tracks->keypoints[1].class_id, 1));
		expect(le(abs(tracks->keypoints[0].pt.x()
					  - (clusters[0].x() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
		expect(le(abs(tracks->keypoints[0].pt.y()
					  - (clusters[0].y() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
		expect(le(abs(tracks->keypoints[1].pt.x()
					  - (clusters[1].x() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
		expect(le(abs(tracks->keypoints[1].pt.y()
					  - (clusters[1].y() + shift * static_cast<float>(newTimestampsAllTracks.size()))),
			std));
	};

	"redetection_strategy"_test = []() {
		// Generate data with timestamp difference of 100ms. Setting redetection strategy to 50ms the new detection
		// should be a key frame

		const cv::Size resolution(640, 480);
		dv::Duration timeWindow = std::chrono::milliseconds(100);

		// Initialize a list of clusters for synthetic data generation
		const std::vector<dv::Point2f> clusters(
			{dv::Point2f(550.f, 400.f), dv::Point2f(70.f, 300.f), dv::Point2f(30.f, 30.f)});

		dv::EventStore events;

		// Generate a batch of normally distributed events around each of the cluster centers
		events += dv::data::generate::normallyDistributedEvents(0, clusters[0], dv::Point2f(3.f, 3.f), 1'000);
		events += dv::data::generate::normallyDistributedEvents(0, clusters[1], dv::Point2f(3.f, 3.f), 1'000);

		const int bandwidth = 20;
		auto meanShift      = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);
		meanShift.accept(events);
		auto tracks = meanShift.runTracking();

		events = dv::EventStore();
		// Generate a batch of normally distributed events around each of the cluster centers
		events += dv::data::generate::normallyDistributedEvents(100, clusters[0], dv::Point2f(3.f, 3.f), 1'000);
		events += dv::data::generate::normallyDistributedEvents(100, clusters[1], dv::Point2f(3.f, 3.f), 1'000);

		meanShift.accept(events);
		tracks = meanShift.runTracking();

		expect(eq(tracks->asKeyFrame, false));
		expect(eq(tracks->keypoints.size(), 2));
		expect(eq(tracks->timestamp, events.getLowestTime()));

		std::unique_ptr<dv::features::RedetectionStrategy> redetection;
		redetection = std::make_unique<dv::features::UpdateIntervalOrFeatureCountRedetection>(
			dv::Duration(std::chrono::milliseconds(33)), 1.f);
		meanShift.setRedetectionStrategy(std::move(redetection));

		auto newEvents = dv::data::generate::normallyDistributedEvents(120, clusters[2], dv::Point2f(3.f, 3.f), 1'000);

		meanShift.accept(newEvents);
		tracks = meanShift.runTracking();
		expect(eq(tracks->asKeyFrame, true));
		expect(eq(tracks->keypoints.size(), 3));
		expect(eq(tracks->timestamp, newEvents.getLowestTime()));
	};
}

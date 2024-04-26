
#include "../../include/dv-processing/features/feature_tracks.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/opencv.hpp>

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace dv::features;
	using namespace std::chrono_literals;

	bool showPreview = false;

	CLI::App app{"Feature tracks visualization test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (showPreview) {
		"display"_test = [] {
			FeatureTracks tracks;

			cv::Mat image(500, 500, CV_8UC3, dv::visualization::colors::black);

			cv::namedWindow("Preview", cv::WINDOW_NORMAL);

			for (int pos = 10; pos < 490; pos += 10) {
				for (int id = 1; id < 49; id++) {
					dv::TimedKeyPoint kpt(dv::Point2f(pos, id * 10.f), 0.f, 0.f, 0.f, 0, id, pos * 10000);
					tracks.accept(kpt);
				}
				cv::Mat preview = tracks.visualize(image);
				cv::imshow("Preview", preview);
				cv::waitKey(20);
			}
		};
	}

	"track_time_limit"_test = [] {
		FeatureTracks tracks;
		tracks.setHistoryDuration(1s);

		expect(eq(tracks.getTrack(0).has_value(), false));
		expect(eq(tracks.isEmpty(), true));
		expect(eq(tracks.getTrackIds().size(), 0ULL));

		dv::TimedKeyPoint kpt(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 0, 0);
		tracks.accept(kpt);

		expect(eq(tracks.getTrackIds().size(), 1ULL));
		expect(eq(tracks.getTrackIds()[0], 0));
		expect(eq(tracks.isEmpty(), false));

		if (auto returnValue = tracks.getTrack(0)) {
			auto track = *returnValue;
			expect(eq(track->size(), 1));
		}
		else {
			expect(eq(tracks.getTrack(0).has_value(), true));
		}

		for (int64_t i = 0; i < 100; i++) {
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 0, i * 100'000));
		}

		if (auto returnValue = tracks.getTrack(0)) {
			auto track = *returnValue;
			expect(eq(track->size(), 11));
		}
		else {
			expect(eq(tracks.getTrack(0).has_value(), true));
		}
	};

	"invalid_arguments"_test = [] {
		expect(throws([] {
			FeatureTracks tracks;
			auto _ = tracks.visualize(cv::Mat());
		}));

		expect(throws([] {
			FeatureTracks tracks;
			tracks.setHistoryDuration(-1000ms);
		}));

		expect(throws([] {
			FeatureTracks tracks;
			tracks.setHistoryDuration(0s);
		}));
	};

	"input_testing"_test = [] {
		FeatureTracks tracks;
		tracks.setHistoryDuration(1s);

		expect(eq(tracks.isEmpty(), true));

		for (int64_t i = 0; i < 100; i++) {
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 0, i * 100'000));
		}
		expect(eq(tracks.isEmpty(), false));
		tracks.clear();
		expect(eq(tracks.isEmpty(), true));

		for (int64_t i = 0; i < 100; i++) {
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 0, i * 100'000));
		}

		if (auto returnValue = tracks.getTrack(0)) {
			auto track = *returnValue;
			expect(eq(track->size(), 11));
			// Limit to half a second duration
			tracks.setHistoryDuration(500ms);
			expect(eq(track->size(), 6));
			// Limit to two seconds duration
			tracks.setHistoryDuration(2s);
			expect(eq(track->size(), 6));
		}
		else {
			expect(eq(tracks.getTrack(0).has_value(), true));
		}

		tracks.clear();
		auto keypoint1 = dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 0, 100'000);
		auto keypoint2 = dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 1, 100'000);
		dv::TimedKeyPointPacket keypoints;
		keypoints.elements.push_back(keypoint1);
		keypoints.elements.push_back(keypoint2);
		tracks.accept(keypoints);
		expect(eq(tracks.getTrackIds().size(), 2));
		expect(eq(tracks.getLatestTrackKeypoints().elements.size(), 2));
	};

	"track_iteration"_test = [] {
		FeatureTracks tracks;
		expect(eq(tracks.isEmpty(), true));
		tracks.setHistoryDuration(10s);
		for (int32_t i = 0; i < 100; i++) {
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, i, i * 100'000));
		}
		expect(eq(tracks.isEmpty(), false));

		int32_t expectedTrackId = 0;

		tracks.eachTrack([&expectedTrackId](const int32_t id, const auto &track) {
			// Expect the track id's to be provided in the same order as it was pushed in above
			expect(eq(id, expectedTrackId));
			expect(eq(track->front().class_id, expectedTrackId));
			expect(eq(track->front().timestamp, expectedTrackId * 100'000));

			expectedTrackId++;
		});
	};

	"stray_track"_test = [] {
		FeatureTracks tracks;
		// Make sure the duration for this test is 1 second
		tracks.setHistoryDuration(1s);
		int64_t timestamp = 0;
		for (int32_t i = 0; i < 100; i++) {
			timestamp = i * 100'000;
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 1, timestamp));
		}
		expect(eq(tracks.isEmpty(), false));

		// Track id 1 is tracked no more, but the time limiting should work nonetheless
		for (int32_t i = 0; i < 5; i++) {
			tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 2, timestamp + (i * 100'000)));
		}

		const auto track1 = *(tracks.getTrack(1));
		const auto track2 = *(tracks.getTrack(2));
		expect(eq(track1->back().timestamp, timestamp));
		expect(eq(track1->size(), 7ULL));
		expect(eq(track2->front().timestamp, timestamp));
		expect(eq(track2->size(), 5ULL));
	};

	"track_timeout_and_get_last_keypoint"_test = [] {
		FeatureTracks tracks;
		// Make sure the duration for this test is 1 second
		tracks.setHistoryDuration(1s);

		expect(!tracks.getTrackTimeout().has_value());

		tracks.setTrackTimeout(200ms);

		expect(tracks.getTrackTimeout().has_value());
		expect(eq(tracks.getTrackTimeout().value().count(), 200'000));

		int64_t timestampIncrement = 100'000;
		int64_t timestamp          = 100'000;

		// Initialize with track id 1
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 1, timestamp));
		expect(tracks.getTrack(1).has_value());
		expect(eq(tracks.getLatestTrackKeypoints().elements.size(), 1));

		// Continue tracking but also adding track with id 2
		timestamp += timestampIncrement;
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 1, timestamp));
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 2, timestamp));
		expect(tracks.getTrack(1).has_value());
		expect(tracks.getTrack(2).has_value());
		expect(eq(tracks.getLatestTrackKeypoints().elements.size(), 2));
		expect(eq(tracks.getLatestTrackKeypoints().elements[0].class_id, 1));
		expect(eq(tracks.getLatestTrackKeypoints().elements[1].class_id, 2));

		// Track 1 is lost, track 2 is still tracked. Track 1 is not immediately lost
		timestamp += timestampIncrement;
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 2, timestamp));
		expect(tracks.getTrack(1).has_value());
		expect(tracks.getTrack(2).has_value());

		// Track 2 is still tracked. Track 1 is still shouldn't be lost, since timeout did not yet exceed
		timestamp += timestampIncrement;
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 2, timestamp));
		expect(tracks.getTrack(1).has_value());
		expect(tracks.getTrack(2).has_value());

		// Track 2 is still tracked. Track 1 should be removed due to track timeout
		timestamp += timestampIncrement;
		tracks.accept(dv::TimedKeyPoint(dv::Point2f(0, 0), 0.f, 0.f, 0.f, 0, 2, timestamp));
		expect(!tracks.getTrack(1).has_value());
		expect(tracks.getTrack(2).has_value());
		expect(eq(tracks.getLatestTrackKeypoints().elements.size(), 1));
		expect(eq(tracks.getLatestTrackKeypoints().elements[0].class_id, 2));
	};

	return EXIT_SUCCESS;
}

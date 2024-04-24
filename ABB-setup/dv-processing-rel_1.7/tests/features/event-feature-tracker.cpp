//
// Created by rokas on 03.08.21.
// Copyright (c) 2021 iniVation AG. All rights reserved.
//

#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/features/event_feature_lk_tracker.hpp"
#include "../../include/dv-processing/features/feature_tracks.hpp"

#include "../utilities/CsvEventReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace std::chrono;

	namespace fs  = std::filesystem;
	namespace dvf = dv::features;
	namespace dvk = dv::kinematics;

	std::string testDataset = "/test_data/processing/hdr_poster";
	std::string outputFileName;
	bool showPreview = false;
	bool speedTest   = false;

	CLI::App app{"Accumulated event feature tracker test"};

	app.add_flag("-s,--speed", speedTest, "Perform speed test by preloading all data into memory");
	app.add_flag("-p,--preview", showPreview, "Display preview image");
	app.add_option("-d,--dataset", testDataset, "Path to testing dataset.");
	app.add_option("-o,--output", outputFileName,
		"Path to an output text file containing track coordinates. If no value is passed, the output file will not be "
		"generated.");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (showPreview) {
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	}

	"tracker_config"_test = [=] {
		cv::Size resolution(100, 100);
		int64_t sampleTimestamp = 1000000LL;

		auto tracker = dvf::EventFeatureLKTracker<>::RegularTracker(resolution);
		// Accumulate every 100ms
		dv::EventStore events = dv::data::generate::eventTestSet(sampleTimestamp, resolution);
		tracker->setFramerate(10);
		tracker->setNumberOfEvents(events.size());

		tracker->accept(events);
		auto result = tracker->runTracking();

		expect(neq(result, nullptr));

		size_t initialCount = result->keypoints.size();
		expect(gt(initialCount, 0ULL));

		// Changing some of the setter functions should not affect the tracking
		tracker->setMotionPredictor(nullptr);
		dv::EventStore sampleEvents = dv::data::generate::eventTestSet(sampleTimestamp + 100000, resolution);
		tracker->accept(sampleEvents);
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));
		tracker->setDetector(nullptr);
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp + 200000, resolution));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setRedetectionStrategy(nullptr);
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp + 300000, resolution));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setAccumulator(nullptr);
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp + 400000, resolution));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setMaxTracks(2 * initialCount);
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp + 500000, resolution));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		if (showPreview) {
			cv::Mat preview;
			cv::cvtColor(tracker->getAccumulatedFrame(), preview, cv::COLOR_GRAY2BGR);
			cv::drawKeypoints(preview, dv::data::fromTimedKeyPoints(result->keypoints), preview);
			cv::imshow("Preview", preview);
			cv::waitKey(0);
		}
	};

	"constant_depth_setting"_test = [] {
		cv::Size resolution(100, 100);
		auto tracker = dvf::EventFeatureLKTracker<>::RegularTracker(resolution);

		// Default is 3.0 meters
		expect(eq(tracker->getConstantDepth(), 3.0_f));

		tracker->setConstantDepth(0.1f);
		expect(eq(tracker->getConstantDepth(), 0.1_f));

		expect(throws([&tracker] {
			tracker->setConstantDepth(-1.f);
		}));
	};

	"tracker_config"_test = [=] {
		int64_t sampleTimestamp = 1000000LL;
		int64_t frameDuration   = 100000LL;

		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		dv::camera::CameraGeometry::SharedPtr camera = std::make_shared<dv::camera::CameraGeometry>(
			resolution.width, resolution.width, resolution.width / 2.f, resolution.height / 2.f, resolution);

		auto tracker          = dvf::EventFeatureLKTracker<dvk::MotionCompensator<>>::MotionAwareTracker(camera);
		dv::EventStore events = dv::data::generate::eventTestSet(sampleTimestamp, resolution);
		tracker->setFramerate(10);
		tracker->setNumberOfEvents(events.size());

		Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
		tracker->accept(dv::kinematics::Transformationf(sampleTimestamp - frameDuration, identity));

		tracker->accept(events);
		tracker->accept(dv::measurements::Depth(sampleTimestamp, 1.f));
		tracker->accept(dv::kinematics::Transformationf(sampleTimestamp, identity));
		auto result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		sampleTimestamp += frameDuration;
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp, resolution));
		tracker->accept(dv::measurements::Depth(sampleTimestamp, 1.f));
		tracker->accept(dv::kinematics::Transformationf(sampleTimestamp, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		sampleTimestamp += frameDuration;
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp, resolution));
		tracker->accept(dv::measurements::Depth(sampleTimestamp, 1.f));
		tracker->accept(dv::kinematics::Transformationf(sampleTimestamp, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		sampleTimestamp += frameDuration;
		tracker->accept(dv::data::generate::eventTestSet(sampleTimestamp, resolution));
		tracker->accept(dv::measurements::Depth(sampleTimestamp, 1.f));
		tracker->accept(dv::kinematics::Transformationf(sampleTimestamp, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		if (showPreview) {
			cv::Mat preview;
			cv::cvtColor(tracker->getAccumulatedFrame(), preview, cv::COLOR_GRAY2BGR);
			cv::drawKeypoints(preview, dv::data::fromTimedKeyPoints(result->keypoints), preview);
			cv::imshow("Preview", preview);
			cv::waitKey(0);
		}
	};

	std::string testFile = testDataset.append("/events.txt");
	if (!fs::exists(testFile)) {
		return EXIT_SUCCESS;
	}

	std::unique_ptr<std::ofstream> outfile = nullptr;
	if (!outputFileName.empty()) {
		outfile = std::make_unique<std::ofstream>(outputFileName);
	}

	"event_tracking"_test = [&] {
		dv::features::ImageFeatureLKTracker::Config config;
		config.numPyrLayers       = 2;
		config.terminationEpsilon = 1e-2;

		cv::Size resolution(240, 180);
		auto det
			= std::make_unique<dv::features::ImagePyrFeatureDetector>(resolution, cv::FastFeatureDetector::create(100));
		auto tracker
			= dv::features::EventFeatureLKTracker<>::RegularTracker(resolution, config, nullptr, std::move(det));
		tracker->setLookbackRejection(true);
		tracker->setRejectionDistanceThreshold(10.f);
		tracker->setFramerate(50);
		tracker->setNumberOfEvents(30000);
		tracker->setMaxTracks(200);

		dv::features::FeatureTracks tracks;
		tracks.setTrackTimeout(30ms);

		boost::lockfree::spsc_queue<dv::EventStore> eventQueue(1000000);
		std::atomic<bool> readerDone{false};
		size_t packetCount = 0;

		std::thread readingThread([&eventQueue, &readerDone, &testFile, &packetCount] {
			test::CsvEventReader reader(testFile);
			while (auto batch = reader.readBatch(2000)) {
				eventQueue.push(*batch);
				packetCount++;
			}
			readerDone = true;
		});
		readingThread.detach();

		// 50ms head start should be more than enough
		if (speedTest) {
			while (!readerDone)
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			std::cout << "Read packets: " << packetCount << std::endl;
		}

		auto start    = high_resolution_clock::now();
		size_t frames = 0;

		while (eventQueue.read_available() > 0 || !readerDone) {
			dv::EventStore batch;
			if (eventQueue.pop(&batch, 1)) {
				tracker->accept(batch);
				while (auto result = tracker->runTracking()) {
					expect(!result->keypoints.empty());
					frames++;
					if (showPreview) {
						tracks.accept(result);
						cv::imshow("Preview", tracks.visualize(tracker->getAccumulatedFrame()));
						cv::waitKey(5);
					}

					if (outfile) {
						for (const auto &keypoint : result->keypoints) {
							*outfile << std::fixed << std::setprecision(6) << keypoint.class_id << " "
									 << static_cast<double>(keypoint.timestamp) * 1e-6 << " " << keypoint.pt.x() << " "
									 << keypoint.pt.y() << std::endl;
						}
					}
				}
			}
		}
		if (speedTest) {
			auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
			std::cout << "Duration: " << duration.count() << " us" << std::endl;
			std::cout << "FPS: " << 1.0 / ((static_cast<double>(duration.count()) * 1e-6) / static_cast<double>(frames))
					  << std::endl;
		}
	};

	return EXIT_SUCCESS;
}

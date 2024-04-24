#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/features/event_combined_lk_tracker.hpp"
#include "../../include/dv-processing/features/feature_tracks.hpp"

#include "../utilities/EventDatasetReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <fstream>
#include <thread>

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace dv::features;

	namespace fs  = std::filesystem;
	namespace dvf = dv::features;

	std::string testDataset = "/test_data/processing/hdr_poster";
	std::string outputFileName;
	bool showPreview = false;

	CLI::App app{"Image feature tracker test"};

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

	"tracker_config"_test = [=] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		dv::camera::CameraGeometry::SharedPtr camera = std::make_shared<dv::camera::CameraGeometry>(
			resolution.width, resolution.width, resolution.width / 2.f, resolution.height / 2.f, resolution);

		auto tracker = dvf::EventCombinedLKTracker<>::MotionAwareTracker(camera);

		Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

		tracker->accept(dv::Frame(0, testImage));
		tracker->accept(dv::data::generate::eventTestSet(0, resolution));
		tracker->accept(dv::data::generate::eventTestSet(500, resolution));
		tracker->accept(dv::measurements::Depth(0, 1.f));
		tracker->accept(dv::kinematics::Transformationf(0, identity));
		auto result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(1000, testImage));
		tracker->accept(dv::data::generate::eventTestSet(1000, resolution));
		tracker->accept(dv::data::generate::eventTestSet(1500, resolution));
		tracker->accept(dv::measurements::Depth(1000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(1000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(2000, testImage));
		tracker->accept(dv::data::generate::eventTestSet(2000, resolution));
		tracker->accept(dv::data::generate::eventTestSet(2500, resolution));
		tracker->accept(dv::measurements::Depth(2000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(2000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(3000, testImage));
		tracker->accept(dv::data::generate::eventTestSet(3000, resolution));
		tracker->accept(dv::data::generate::eventTestSet(3500, resolution));
		tracker->accept(dv::measurements::Depth(3000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(3000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));
	};

	"bad_timestamping"_test = [=] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::EventCombinedLKTracker<>::RegularTracker(resolution);
		tracker->accept(dv::Frame(0, testImage));
		tracker->accept(dv::data::generate::eventTestSet(0, resolution));
		tracker->accept(dv::data::generate::eventTestSet(500, resolution));
		auto result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(1000, testImage));
		tracker->accept(dv::data::generate::eventTestSet(1000, resolution));
		tracker->accept(dv::data::generate::eventTestSet(1500, resolution));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		expect(throws([&tracker] {
			tracker->accept(dv::Frame(0, cv::Mat()));
		}));
		expect(throws([&tracker, &resolution] {
			tracker->accept(dv::data::generate::eventTestSet(100, resolution));
		}));
	};

	"no_events_tracking"_test = [] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::EventCombinedLKTracker<>::RegularTracker(resolution);

		// Let's track just by passing in the image without events
		tracker->accept(dv::Frame(0, testImage));
		auto result1 = tracker->runTracking();
		tracker->accept(dv::Frame(10'000, testImage));
		auto result2 = tracker->runTracking();
		tracker->accept(dv::Frame(20'000, testImage));
		auto result3 = tracker->runTracking();
		tracker->accept(dv::Frame(30'000, testImage));
		auto result4 = tracker->runTracking();

		// Even without events you should expect successful tracking across frames
		expect(eq(result1->keypoints.size(), result2->keypoints.size()));
		expect(eq(result2->keypoints.size(), result3->keypoints.size()));
		expect(eq(result3->keypoints.size(), result4->keypoints.size()));

		for (size_t i = 0; i < result1->keypoints.size(); i++) {
			expect(eq(result1->keypoints.at(i).pt.x(), result2->keypoints.at(i).pt.x()));
			expect(eq(result2->keypoints.at(i).pt.x(), result3->keypoints.at(i).pt.x()));
			expect(eq(result3->keypoints.at(i).pt.x(), result4->keypoints.at(i).pt.x()));
			expect(eq(result1->keypoints.at(i).pt.y(), result2->keypoints.at(i).pt.y()));
			expect(eq(result2->keypoints.at(i).pt.y(), result3->keypoints.at(i).pt.y()));
			expect(eq(result3->keypoints.at(i).pt.y(), result4->keypoints.at(i).pt.y()));
			expect(eq(result1->keypoints.at(i).class_id, result2->keypoints.at(i).class_id));
			expect(eq(result2->keypoints.at(i).class_id, result3->keypoints.at(i).class_id));
			expect(eq(result3->keypoints.at(i).class_id, result4->keypoints.at(i).class_id));
		}
	};

	"robustness_to_noise"_test = [] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::EventCombinedLKTracker<>::RegularTracker(resolution);

		const int64_t timeIncrement = 100;
		const int64_t finalTime     = 30'000;
		const int64_t slices        = finalTime / timeIncrement;
		const size_t numEvents      = 300;

		dv::EventStore events;

		// Generate uniformly distributed events at random pixels every timeIncrement period
		// Seed is controllably modified to have events at different locations in reproducible way
		for (int64_t seed = 1; seed < slices; seed++) {
			events.add(
				dv::data::generate::uniformlyDistributedEvents(seed * timeIncrement, resolution, numEvents, seed));
		}

		tracker->accept(events);

		tracker->accept(dv::Frame(0, testImage));
		auto result1 = tracker->runTracking();
		tracker->accept(dv::Frame(10'000, testImage));
		auto result2 = tracker->runTracking();
		tracker->accept(dv::Frame(20'000, testImage));
		auto result3 = tracker->runTracking();
		tracker->accept(dv::Frame(finalTime, testImage));
		auto result4 = tracker->runTracking();

		// Even with just pure noise in the background we expect to track everything successfully
		expect(gt(result1->keypoints.size(), 0));
		expect(gt(result2->keypoints.size(), 0));
		expect(gt(result3->keypoints.size(), 0));
		expect(gt(result4->keypoints.size(), 0));

		expect(eq(result1->keypoints.size(), result2->keypoints.size()));
		expect(eq(result2->keypoints.size(), result3->keypoints.size()));
		expect(eq(result3->keypoints.size(), result4->keypoints.size()));

		for (size_t i = 0; i < result2->keypoints.size(); i++) {
			// Rounding the coordinates, because noise in the event modality can shift the tracks at a
			// subpixel-precision
			expect(eq(std::round(result1->keypoints.at(i).pt.x()), std::round(result2->keypoints.at(i).pt.x())));
			expect(eq(std::round(result2->keypoints.at(i).pt.x()), std::round(result3->keypoints.at(i).pt.x())));
			expect(eq(std::round(result3->keypoints.at(i).pt.x()), std::round(result4->keypoints.at(i).pt.x())));
			expect(eq(std::round(result1->keypoints.at(i).pt.y()), std::round(result2->keypoints.at(i).pt.y())));
			expect(eq(std::round(result2->keypoints.at(i).pt.y()), std::round(result3->keypoints.at(i).pt.y())));
			expect(eq(std::round(result3->keypoints.at(i).pt.y()), std::round(result4->keypoints.at(i).pt.y())));
			expect(eq(result1->keypoints.at(i).class_id, result2->keypoints.at(i).class_id));
			expect(eq(result2->keypoints.at(i).class_id, result3->keypoints.at(i).class_id));
			expect(eq(result3->keypoints.at(i).class_id, result4->keypoints.at(i).class_id));
		}

		const auto accumulatedFrames = tracker->getAccumulatedFrames();
		expect(!accumulatedFrames.empty());
	};

	"event_rate_threshold"_test = [] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::EventCombinedLKTracker<>::RegularTracker(resolution);

		const int64_t timeIncrement = 100;
		const int64_t finalTime     = 30'000;
		const int64_t slices        = finalTime / timeIncrement;
		const size_t numEvents      = 300;

		dv::EventStore events;

		// Generate uniformly distributed events at random pixels every timeIncrement period
		// Seed is controllably modified to have events at different locations in reproducible way
		for (int64_t seed = 1; seed < slices; seed++) {
			events.add(
				dv::data::generate::uniformlyDistributedEvents(seed * timeIncrement, resolution, numEvents, seed));
		}

		tracker->accept(events);
		// Setting event rate too high - the event data should be ignored
		tracker->setMinRateForIntermediateTracking(events.rate() * 2.0);

		tracker->accept(dv::Frame(0, testImage));
		auto result1 = tracker->runTracking();
		tracker->accept(dv::Frame(10'000, testImage));
		auto result2 = tracker->runTracking();
		tracker->accept(dv::Frame(20'000, testImage));
		auto result3 = tracker->runTracking();
		tracker->accept(dv::Frame(finalTime, testImage));
		auto result4 = tracker->runTracking();

		// Even without events you should expect successful tracking across frames
		expect(eq(result1->keypoints.size(), result2->keypoints.size()));
		expect(eq(result2->keypoints.size(), result3->keypoints.size()));
		expect(eq(result3->keypoints.size(), result4->keypoints.size()));

		for (size_t i = 0; i < result1->keypoints.size(); i++) {
			expect(eq(result1->keypoints.at(i).pt.x(), result2->keypoints.at(i).pt.x()));
			expect(eq(result2->keypoints.at(i).pt.x(), result3->keypoints.at(i).pt.x()));
			expect(eq(result3->keypoints.at(i).pt.x(), result4->keypoints.at(i).pt.x()));
			expect(eq(result1->keypoints.at(i).pt.y(), result2->keypoints.at(i).pt.y()));
			expect(eq(result2->keypoints.at(i).pt.y(), result3->keypoints.at(i).pt.y()));
			expect(eq(result3->keypoints.at(i).pt.y(), result4->keypoints.at(i).pt.y()));
			expect(eq(result1->keypoints.at(i).class_id, result2->keypoints.at(i).class_id));
			expect(eq(result2->keypoints.at(i).class_id, result3->keypoints.at(i).class_id));
			expect(eq(result3->keypoints.at(i).class_id, result4->keypoints.at(i).class_id));
		}

		// No event accumulated frames here due to minEventRate configuration
		const auto accumulatedFrames = tracker->getAccumulatedFrames();
		expect(accumulatedFrames.empty());
	};

	"constant_depth_setting"_test = [] {
		cv::Size resolution(100, 100);
		auto tracker = dvf::EventCombinedLKTracker<>::RegularTracker(resolution);

		// Default is 3.0 meters
		expect(eq(tracker->getConstantDepth(), 3.0_f));

		tracker->setConstantDepth(0.1f);
		expect(eq(tracker->getConstantDepth(), 0.1_f));

		expect(throws([&tracker] {
			tracker->setConstantDepth(-1.f);
		}));
	};

	if (!fs::exists(testDataset)) {
		return EXIT_SUCCESS;
	}

	std::unique_ptr<std::ofstream> outfile = nullptr;
	if (!outputFileName.empty()) {
		outfile = std::make_unique<std::ofstream>(outputFileName);
	}

	if (showPreview) {
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
		cv::waitKey(5);
	}

	cv::Size resolution(240, 180);
	auto tracker = dv::features::EventCombinedLKTracker<>::RegularTracker(resolution);

	typedef std::pair<std::pair<int64_t, cv::Mat>, dv::EventStore> InputType;
	boost::lockfree::spsc_queue<InputType> inputQueue(10000);
	std::atomic<bool> readerDone{false};

	std::thread readingThread([&inputQueue, &readerDone, &testDataset] {
		test::EventDatasetReader reader(testDataset);
		while (auto image = reader.readNextImage()) {
			dv::EventStore events = reader.readEventsUntil(image->first);
			inputQueue.push(std::make_pair(*image, events));
		}
		readerDone = true;
	});
	readingThread.detach();

	// 50ms head start should be more than enough
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	size_t redetections       = 0;
	size_t lostTracks         = 0;
	size_t lastFrameTrackSize = 0;
	dv::features::EventCombinedLKTracker<>::Result previousResult;

	dv::features::FeatureTracks frameTrackHistory;

	while (inputQueue.read_available() > 0 || !readerDone) {
		InputType batch;
		if (inputQueue.pop(&batch, 1) > 0) {
			tracker->accept(dv::Frame(batch.first.first, batch.first.second));
			tracker->accept(batch.second);
			if (batch.second.isEmpty()) {
				break;
			}
			while (auto result = tracker->runTracking()) {
				expect(result != nullptr);

				if (result->asKeyFrame) {
					redetections++;
				}
				else {
					if (lastFrameTrackSize > 0) {
						lostTracks += lastFrameTrackSize - result->keypoints.size();
					}
					lastFrameTrackSize = result->keypoints.size();
				}

				if (outfile) {
					for (const auto &keypoint : result->keypoints) {
						*outfile << std::fixed << std::setprecision(6) << keypoint.class_id << " "
								 << static_cast<double>(keypoint.timestamp) * 1e-6 << " " << keypoint.pt.x() << " "
								 << keypoint.pt.y() << std::endl;
					}
				}

				if (showPreview) {
					cv::Mat preview;
					cv::Mat colorImage;
					cv::Mat colorFrame;
					cv::cvtColor(batch.first.second, colorFrame, cv::COLOR_GRAY2BGR);
					auto eventFrames = tracker->getAccumulatedFrames();
					if (!eventFrames.empty()) {
						auto eventTracks = tracker->getEventTrackPoints();
						cv::cvtColor(eventFrames.back().pyramid[0], colorImage, cv::COLOR_GRAY2BGR);
						for (size_t index = 0; index < (eventTracks.back().size() - 1); index++) {
							const auto &point = eventTracks.back()[index];
							cv::drawMarker(colorImage, point, cv::Scalar(255, 0, 255), cv::MARKER_SQUARE, 5);
							cv::drawMarker(colorFrame, point, cv::Scalar(0, 255, 255), cv::MARKER_SQUARE, 5);
						}

						for (const auto &currentPoint : result->keypoints) {
							auto &currentKeyPointId = currentPoint.class_id;
							for (const auto &oldPoints : previousResult.keypoints) {
								if (oldPoints.class_id == currentKeyPointId) {
									cv::arrowedLine(colorFrame, *reinterpret_cast<const cv::Point2f *>(&oldPoints.pt),
										*reinterpret_cast<const cv::Point2f *>(&currentPoint.pt),
										cv::Scalar(0, 255, 255), 1);
									break;
								}
							}
						}

						if (!colorImage.empty()) {
							frameTrackHistory.accept(result);
							cv::vconcat(
								std::vector<cv::Mat>({frameTrackHistory.visualize(colorFrame), colorImage}), preview);
							cv::imshow("Preview", preview);
							cv::waitKey(5);
						}
					}
					previousResult = *result;
				}
			}
		}
	}

	std::cout << "Redetections: " << redetections << std::endl;
	std::cout << "Number of lost tracks: " << lostTracks << std::endl;

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/features/feature_tracks.hpp"
#include "../../include/dv-processing/features/image_feature_lk_tracker.hpp"

#include "../utilities/EventDatasetReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <thread>

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace std::chrono_literals;
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

	if (fs::exists(testDataset)) {
		"run_dataset"_test = [=] {
			std::unique_ptr<dv::features::ImageFeatureLKTracker> tracker = nullptr;
			std::unique_ptr<std::ofstream> outfile                       = nullptr;
			if (!outputFileName.empty()) {
				outfile = std::make_unique<std::ofstream>(outputFileName);
			}

			if (showPreview) {
				cv::namedWindow("Preview", cv::WINDOW_NORMAL);
			}

			typedef std::pair<int64_t, cv::Mat> InputType;
			boost::lockfree::spsc_queue<InputType> inputQueue(10000);
			std::atomic<bool> readerDone{false};

			std::thread readingThread([&inputQueue, &readerDone, &testDataset] {
				test::EventDatasetReader reader(testDataset);
				while (auto image = reader.readNextImage()) {
					inputQueue.push(*image);
				}
				readerDone = true;
			});
			readingThread.detach();

			std::this_thread::sleep_for(std::chrono::milliseconds(50));

			size_t redetections       = 0;
			size_t lostTracks         = 0;
			size_t lastFrameTrackSize = 0;

			while (inputQueue.read_available() > 0 || !readerDone) {
				InputType batch;
				if (inputQueue.pop(&batch, 1) > 0) {
					if (!tracker) {
						tracker = dv::features::ImageFeatureLKTracker::RegularTracker(batch.second.size());
					}

					tracker->accept(dv::Frame(batch.first, batch.second));
					auto result = tracker->runTracking();
					expect(neq(result, nullptr));

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
						cv::drawKeypoints(batch.second, dv::data::fromTimedKeyPoints(result->keypoints), preview,
							cv::Scalar(255, 0, 255));
						cv::imshow("Preview", preview);
						cv::waitKey(5);
					}
				}
			}
		};
	}

	"tracker_config"_test = [=] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::ImageFeatureLKTracker::RegularTracker(resolution);
		tracker->accept(dv::Frame(0, testImage));
		auto result = tracker->runTracking();

		size_t initialCount = result->keypoints.size();
		expect(gt(initialCount, 0ULL));

		// Changing some of the setter functions should not affect the tracking
		tracker->setMotionPredictor(nullptr);
		tracker->accept(dv::Frame(1, testImage));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setDetector(nullptr);
		tracker->accept(dv::Frame(2, testImage));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setRedetectionStrategy(nullptr);
		tracker->accept(dv::Frame(3, testImage));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		tracker->setMaxTracks(2 * initialCount);
		tracker->accept(dv::Frame(4, testImage));
		result = tracker->runTracking();
		expect(eq(result->keypoints.size(), initialCount));

		if (showPreview) {
			cv::Mat preview;
			cv::cvtColor(testImage, preview, cv::COLOR_GRAY2BGR);
			cv::drawKeypoints(preview, dv::data::fromTimedKeyPoints(result->keypoints), preview);
			cv::imshow("Preview", preview);
			cv::waitKey(0);
		}
	};

	"tracker_config"_test = [=] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		dv::camera::CameraGeometry::SharedPtr camera = std::make_shared<dv::camera::CameraGeometry>(
			resolution.width, resolution.width, resolution.width / 2.f, resolution.height / 2.f, resolution);

		auto tracker = dvf::ImageFeatureLKTracker::MotionAwareTracker(camera);

		Eigen::Matrix4f identity;
		identity.setIdentity();

		tracker->accept(dv::Frame(0, testImage));
		tracker->accept(dv::measurements::Depth(0, 1.f));
		tracker->accept(dv::kinematics::Transformationf(0, identity));
		auto result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(1000, testImage));
		tracker->accept(dv::measurements::Depth(1000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(1000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(2000, testImage));
		tracker->accept(dv::measurements::Depth(2000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(2000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(3000, testImage));
		tracker->accept(dv::measurements::Depth(3000, 1.f));
		tracker->accept(dv::kinematics::Transformationf(3000, identity));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));
	};

	"bad_timestamping"_test = [=] {
		cv::Size resolution(100, 100);
		cv::Mat testImage = dv::data::generate::sampleImage(resolution);

		auto tracker = dvf::ImageFeatureLKTracker::RegularTracker(resolution);
		tracker->accept(dv::Frame(0, testImage));
		auto result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		tracker->accept(dv::Frame(1000, testImage));
		result = tracker->runTracking();
		expect(gt(result->keypoints.size(), 0));

		expect(throws([&tracker] {
			tracker->accept(dv::Frame(0, cv::Mat()));
		}));
	};

	"constant_depth_setting"_test = [] {
		cv::Size resolution(100, 100);
		auto tracker = dvf::ImageFeatureLKTracker::RegularTracker(resolution);

		// Default is 3.0 meters
		expect(eq(tracker->getConstantDepth(), 3.0_f));

		tracker->setConstantDepth(0.1f);
		expect(eq(tracker->getConstantDepth(), 0.1_f));

		expect(throws([&tracker] {
			tracker->setConstantDepth(-1.f);
		}));
	};

	"invalid_resolution"_test = [] {
		const cv::Size resolution(100, 100);
		auto tracker = dvf::ImageFeatureLKTracker::RegularTracker(resolution);

		// Does not throw exceptions if image resolution is correct
		expect(nothrow([&tracker] {
			tracker->accept(dv::Frame(0, cv::Mat::zeros(100, 100, CV_8UC1)));
		}));

		// Throws exception if image resolution is incorrect
		expect(throws([&tracker] {
			tracker->accept(dv::Frame(0, cv::Mat::zeros(200, 200, CV_8UC1)));
		}));
	};

	return EXIT_SUCCESS;
}

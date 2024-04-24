#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/features/arc_corner_detector.hpp"
#include "../../include/dv-processing/features/feature_detector.hpp"
#include "../../include/dv-processing/features/image_pyramid.hpp"

#include "../utilities/EventDatasetReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>

using SimpleTimeSurface = std::vector<std::vector<int64_t>>;

dv::EventStore simpleTimeSurfaceToEventStore(const SimpleTimeSurface &ts, const bool polarity = true) {
	std::vector<dv::Event> flattenedTs;

	for (size_t x = 0; x < ts[0].size(); x++) {
		for (size_t y = 0; y < ts.size(); y++) {
			flattenedTs.emplace_back(ts[y][x], x, y, polarity);
		}
	}

	std::sort(flattenedTs.begin(), flattenedTs.end(), [](const auto &e1, const auto &e2) {
		return e1.timestamp() < e2.timestamp();
	});

	dv::EventStore events;

	for (const auto &e : flattenedTs) {
		events << e;
	}

	return events;
}

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace dv;
	using namespace dv::features;
	namespace fs = std::filesystem;

	CLI::App app{"Arc corner detector test"};

	std::string testDataset;
	std::string outputFileName;
	bool showPreview = false;

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	app.add_option("-d,--dataset", testDataset, "Path to testing dataset.");
	app.add_option("-o,--output", outputFileName,
		"Path to an output text file containing keypoint coordinates. If no value is passed, the output file will not "
		"be generated.");

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (!testDataset.empty() && fs::exists(testDataset)) {
		std::unique_ptr<std::ofstream> outfile = nullptr;
		if (!outputFileName.empty()) {
			outfile = std::make_unique<std::ofstream>(outputFileName);
		}

		if (showPreview) {
			cv::namedWindow("Preview", cv::WINDOW_NORMAL);
			cv::waitKey(1);
		}

		const cv::Size resolution{240, 180};

		using Arc = ArcCornerDetector<dv::SpeedInvariantTimeSurfaceBase<dv::EventStore, 14>, 5, 6>;
		auto arc  = std::make_shared<Arc>(resolution, 70, false);

		using ArcDetector    = dv::features::FeatureDetector<dv::EventStore, Arc>;
		using CvFastDetector = dv::features::FeatureDetector<dv::features::ImagePyramid, cv::Feature2D>;

		ArcDetector arcDetector{resolution, arc, ArcDetector::FeaturePostProcessing::AdaptiveNMS, 0.05f};
		CvFastDetector cvFastDetector{resolution, cv::FastFeatureDetector::create(100, false),
			CvFastDetector::FeaturePostProcessing::AdaptiveNMS, 0.05f};

		test::EventDatasetReader reader{testDataset};

		dv::EdgeMapAccumulator acc{resolution, 0.2f};

		while (auto events = reader.readEventBatch(10000)) {
			const auto cornersFromArc = arcDetector.runDetection(*events, 30);

			acc << *events;

			dv::Frame timedFrame = acc.generateFrame();

			const auto cornersFromCvFast
				= cvFastDetector.runDetection(dv::features::ImagePyramid(0LL, timedFrame.image), 30);

			if (showPreview) {
				dv::Frame frameFromArc = arc->getTimeSurface(1).generateFrame();
				cv::drawKeypoints(frameFromArc.image, dv::data::fromTimedKeyPoints(cornersFromArc), frameFromArc.image);
				cv::putText(
					frameFromArc.image, "Arc*", cv::Point(15, 15), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar::all(0xff));

				cv::drawKeypoints(timedFrame.image, dv::data::fromTimedKeyPoints(cornersFromCvFast), timedFrame.image);
				cv::putText(timedFrame.image, "cv FAST", cv::Point(15, 15), cv::FONT_HERSHEY_DUPLEX, 0.5,
					cv::Scalar::all(0xff));

				frameFromArc.image.push_back(timedFrame.image);

				cv::imshow("Preview", frameFromArc.image);
				cv::waitKey(50);
			}

			if (outfile) {
				for (const auto &keypoint : cornersFromArc) {
					*outfile << std::fixed << std::setprecision(6) << keypoint.class_id << " "
							 << static_cast<double>(keypoint.timestamp) * 1e-6 << " " << keypoint.pt.x() << " "
							 << keypoint.pt.y() << std::endl;
				}
			}
		}
	}

	"simple"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(eq(corners1.size(), 1));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 1));
		expect(corners[0].pt == dv::Point2f(resolution.width / 2, resolution.height / 2));
		expect(eq(corners[0].timestamp, 10000));
		expect(eq(corners[0].response, 10000));
	};

	"mask"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(eq(corners1.size(), 1));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, true);

		cv::Mat mask{resolution, CV_8U, cv::Scalar::all(0)};
		const auto corners = detector.runDetection(events2, 1, mask);

		expect(eq(corners.size(), 0));
	};

	"acute-angle"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000,  10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000,  10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 100000, 10000, 0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000,  0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,      0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(gt(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 1));
		expect(corners[0].pt == dv::Point2f(resolution.width / 2, resolution.height / 2));
		expect(eq(corners[0].timestamp, 10000));
		expect(eq(corners[0].response, 10000));
	};

	"too-acute-angle"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0, 0, 0}
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(eq(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 0));
	};

	"blunt-angle"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }
        };
		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(gt(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 1));
		expect(corners[0].pt == dv::Point2f(resolution.width / 2, resolution.height / 2));
		expect(eq(corners[0].timestamp, 10000));
		expect(eq(corners[0].response, 10000));
	};

	"too-blunt-angle"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     10000}
        };
		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(gt(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 0));
	};

	"non-contiguous-arc"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 0,     10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 0,     10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 0,     10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 0,     10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 0,     10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 0,     0,     0,     0,     0,     0,     0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(ge(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 0));
	};

	"not-consistently-greater"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 0,     0,     0,     0,     0,     0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(ge(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 0));
	};

	"response"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }, //
				{0, 0, 0, 0, 0, 0, 0, 0, 8000,  0,     0,     0,     0,     0,     0,     0    }
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(ge(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 1));
		expect(corners[0].pt == dv::Point2f(resolution.width / 2, resolution.height / 2));
		expect(eq(corners[0].timestamp, 10000));
		expect(eq(corners[0].response, 2000));
	};

	"no-slope"_test = []() {
		const SimpleTimeSurface ts = //
			{
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}, //
				{10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000,
                 10000, 10000}
        };

		const cv::Size resolution{static_cast<int32_t>(ts[0].size()), static_cast<int32_t>(ts.size())};

		using Arc = ArcCornerDetector<TimeSurface, 3, 4>;
		FeatureDetector<EventStore, Arc> detector{resolution, std::make_shared<Arc>(resolution, 50, false),
			FeatureDetector<EventStore, Arc>::FeaturePostProcessing::None, 0.25};

		const auto events = simpleTimeSurfaceToEventStore(ts);

		const auto corners1 = detector.runDetection(events, 1);
		expect(eq(corners1.size(), 0));

		dv::EventStore events2;
		events2 << dv::Event(10000, resolution.width / 2, resolution.height / 2, 1);

		const auto corners = detector.runDetection(events2, 1);

		expect(eq(corners.size(), 0));
	};

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/depth/semi_dense_stereo_matcher.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

std::tuple<dv::EventStore, dv::EventStore, dv::EventStore> generateTestData() {
	const cv::Size resolution(640, 480);
	const auto events = dv::data::generate::eventTestSet(1, resolution);
	// Use the "plus" operator to have multiple events at the same location, to get higher contrast in accumulation
	const auto left = events + events + events + events;
	dv::EventStore right;
	for (const auto event : left) {
		// Emulate disparity of 10 pixels
		right.emplace_back(event.timestamp(), event.x() - 10, event.y(), event.polarity());
	}

	return std::make_tuple(events, left, right);
}

int main(int ac, char **av) {
	using namespace boost::ut;

	bool showPreview = false;

	CLI::App app{"Disparity estimation test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	"unset_parameters"_test = [] {
		const cv::Size resolution(640, 480);
		dv::SemiDenseStereoMatcher matcher(resolution, resolution);

		expect(throws([&matcher] {
			const auto _ = matcher.estimateDepth(cv::Mat(), dv::EventStore());
		}));

		expect(throws([&matcher] {
			const auto _ = matcher.estimateDepthFrame(cv::Mat());
		}));
	};

	"event_disparity"_test = [showPreview] {
		const cv::Size resolution(640, 480);
		dv::EventStore events, left, right;
		std::tie(events, left, right) = generateTestData();

		dv::SemiDenseStereoMatcher matcher(resolution, resolution);
		cv::Mat disparity = matcher.computeDisparity(left, right);

		if (showPreview) {
			cv::Mat disparityU8, preview;
			disparity.convertTo(disparityU8, CV_8UC1);
			cv::Mat leftImage = matcher.getLeftFrame().image;
			cv::hconcat(leftImage, disparityU8, preview);
			cv::imshow("disparity", preview);
			cv::waitKey(0);
		}

		for (const auto &event : events) {
			const auto disparityValue = static_cast<float>(disparity.at<uint16_t>(event.y(), event.x())) / 16.f;
			// Expect disparity value at those locations to be roughly 10
			expect(eq(disparityValue, 10._f));
		}
	};

	"depth_estimation"_test = [] {
		const cv::Size resolution(640, 480);
		dv::EventStore events, left, right;
		std::tie(events, left, right) = generateTestData();

		// Some geometry info
		dv::camera::CameraGeometry idealGeometry(static_cast<float>(resolution.width) * 0.5f,
			static_cast<float>(resolution.width) * 0.5f, static_cast<float>(resolution.width) * 0.5f,
			static_cast<float>(resolution.height) * 0.5f, resolution);
		std::vector<float> transformation = std::vector<float>(16, 0.f);
		transformation[0]                 = 1.0f;
		transformation[5]                 = 1.0f;
		transformation[10]                = 1.0f;
		transformation[15]                = 1.0f;

		// Two cameras 20cm apart
		transformation[3] = -0.2f;

		dv::camera::StereoGeometry geometry(idealGeometry, idealGeometry, transformation);
		dv::SemiDenseStereoMatcher matcher(std::make_unique<dv::camera::StereoGeometry>(geometry));
		cv::Mat disparity = matcher.computeDisparity(left, right);

		cv::Mat undistorted;

		const dv::DepthFrame depthFrame = matcher.estimateDepthFrame(disparity);
		expect(!depthFrame.depth.empty());
		const dv::DepthEventStore depthEvents = matcher.estimateDepth(disparity, events);
		expect(!depthEvents.isEmpty());
		for (const auto &event : depthEvents) {
			const uint16_t depth = depthFrame.depth.at((event.y() * depthFrame.sizeX) + event.x());

			// OpenCV 4.8.0 issue: https://gitlab.com/inivation/dv/internal/dv-processing-internal/-/issues/118
#if (CV_VERSION_MAJOR <= 4) && (CV_VERSION_MINOR <= 7)
			expect(eq(depth, event.depth()));
#endif
		}
	};

	return EXIT_SUCCESS;
}

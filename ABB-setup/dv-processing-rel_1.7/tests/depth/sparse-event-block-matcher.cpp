#include "../../include/dv-processing/camera/calibration_set.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/depth/sparse_event_block_matcher.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

using namespace boost::ut;

dv::EventStore generateTestPatch(const cv::Rect &region) {
	cv::Point2f top(region.x + region.width * 0.333f, region.y);
	cv::Point2f middle(region.x + region.width * 0.666f, region.y + region.height * 0.5);
	cv::Point2f bottom(region.x + region.width * 0.333f, region.y + region.height);

	dv::EventStore events;
	events.add(dv::data::generate::eventLine(0, top, middle));
	events.add(dv::data::generate::eventLine(0, middle, bottom));
	return events;
}

int main(int ac, char **av) {
	bool showPreview = false;

	CLI::App app{"Disparity estimation test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (showPreview) {
		cv::namedWindow("preview", cv::WINDOW_NORMAL);
	}

	"block_matching"_test = [showPreview] {
		const cv::Size resolution(100, 100);
		dv::EventStore left, right;
		dv::SparseEventBlockMatcher matcher(resolution, cv::Size(24, 24), 40, 0, 1.0f);
		dv::cvector<cv::Point2i> coords;
		coords.emplace_back(50, 50);
		coords.emplace_back(25, 25);
		coords.emplace_back(75, 75);
		coords.emplace_back(80, 80);
		coords.emplace_back(24, 24);

		auto disparities = matcher.computeDisparitySparse(left, right, coords);
		expect(eq(disparities.size(), coords.size()));
		for (size_t i = 0; i < disparities.size(); i++) {
			expect(!disparities[i].valid);
			expect(eq(disparities[i].coordinates.x, coords[i].x));
			expect(eq(disparities[i].coordinates.y, coords[i].y));
		}

		left = generateTestPatch(cv::Rect(38, 38, 24, 24));

		disparities = matcher.computeDisparitySparse(left, right, coords);
		expect(eq(disparities.size(), coords.size()));
		for (size_t i = 0; i < disparities.size(); i++) {
			expect(!disparities[i].valid);
			expect(eq(disparities[i].coordinates.x, coords[i].x));
			expect(eq(disparities[i].coordinates.y, coords[i].y));
		}

		disparities = matcher.computeDisparitySparse(right, left, coords);
		expect(eq(disparities.size(), coords.size()));
		for (size_t i = 0; i < disparities.size(); i++) {
			expect(!disparities[i].valid);
			expect(eq(disparities[i].coordinates.x, coords[i].x));
			expect(eq(disparities[i].coordinates.y, coords[i].y));
		}

		for (const auto &ev : left) {
			right.emplace_back(ev.timestamp(), ev.x() - 10, ev.y(), ev.polarity());
		}

		disparities = matcher.computeDisparitySparse(left, right, coords);
		expect(eq(disparities.size(), coords.size()));
		for (size_t i = 0; i < disparities.size(); i++) {
			if (i == 0) {
				expect(disparities[i].valid);
				expect(eq(disparities[i].coordinates.x, coords[i].x));
				expect(eq(disparities[i].coordinates.y, coords[i].y));
				expect(eq(disparities[i].disparity.value(), 10));
				expect(disparities[i].correlation.has_value());
				expect(disparities[i].score.has_value());
			}
			else {
				expect(!disparities[i].valid);
				expect(eq(disparities[i].coordinates.x, coords[i].x));
				expect(eq(disparities[i].coordinates.y, coords[i].y));
			}
		}
		if (showPreview) {
			cv::Mat leftIm  = matcher.getLeftFrame().image;
			cv::Mat rightIm = matcher.getRightFrame().image;
			cv::rectangle(leftIm, cv::Rect(38, 38, 24, 24), cv::Scalar(255));
			cv::rectangle(rightIm, cv::Rect(38 - disparities[0].disparity.value(), 38, 24, 24), cv::Scalar(255));
			cv::Mat preview;
			cv::hconcat(matcher.getLeftFrame().image, matcher.getRightFrame().image, preview);
			cv::imshow("preview", preview);
			cv::waitKey();
		}
	};

	"matcher_configuration"_test = [] {
		const cv::Size resolution(640, 480);
		dv::SparseEventBlockMatcher matcher(resolution, cv::Size(24, 24), 40, 0, 1.0f);

		cv::Point2i poiCenter(resolution.width / 2, resolution.height / 2);
		dv::cvector<cv::Point2i> coords;
		coords.push_back(poiCenter);

		dv::EventStore left = generateTestPatch(cv::Rect(poiCenter.x - 12, poiCenter.y - 12, 24, 24));
		dv::EventStore right;
		for (const auto &ev : left) {
			right.emplace_back(ev.timestamp(), ev.x() - 10, ev.y(), ev.polarity());
		}

		// Vary minimum disparity and validate matching output
		for (int32_t value = 0; value < 20; value++) {
			matcher.setMinDisparity(value);
			auto disparities = matcher.computeDisparitySparse(left, right, coords);
			expect(eq(disparities.size(), 1));

			if (value <= 10) {
				expect(disparities[0].valid);
				expect(disparities[0].correlation.has_value());
				expect(disparities[0].disparity.has_value());
				expect(disparities[0].score.has_value());
				expect(eq(disparities[0].coordinates.x, coords[0].x));
				expect(eq(disparities[0].coordinates.y, coords[0].y));
				expect(eq(disparities[0].disparity.value(), 10));
			}
			else {
				expect(!disparities[0].valid);
				expect(eq(disparities[0].coordinates.x, coords[0].x));
				expect(eq(disparities[0].coordinates.y, coords[0].y));
			}
		}

		// Restore original settings
		matcher.setMinDisparity(0);

		// Vary maximum disparity and validate output
		for (int32_t value = 40; value > 5; value--) {
			matcher.setMaxDisparity(value);

			if (value >= 10) {
				auto disparities = matcher.computeDisparitySparse(left, right, coords);
				expect(eq(disparities.size(), 1));
				expect(disparities[0].valid);
				expect(disparities[0].correlation.has_value());
				expect(disparities[0].disparity.has_value());
				expect(disparities[0].score.has_value());
				expect(eq(disparities[0].coordinates.x, coords[0].x));
				expect(eq(disparities[0].coordinates.y, coords[0].y));
				expect(eq(disparities[0].disparity.value(), 10));
			}
			else {
				auto disparities = matcher.computeDisparitySparse(left, right, coords);
				expect(eq(disparities.size(), 1));
				expect(!disparities[0].valid);
				expect(eq(disparities[0].coordinates.x, coords[0].x));
				expect(eq(disparities[0].coordinates.y, coords[0].y));
			}
		}
	};

	"invalid_configurations"_test = [] {
		expect(throws([] {
			dv::camera::CameraGeometry a(100.f, 100.f, 50.f, 50.f, cv::Size(100, 100));
			dv::camera::CameraGeometry b(100.f, 100.f, 50.f, 50.f, cv::Size(100, 200));
			dv::SparseEventBlockMatcher matcher(std::make_unique<dv::camera::StereoGeometry>(
				a, b, std::vector<float>({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1})));
		}));

		expect(throws([] {
			dv::camera::CameraGeometry a(100.f, 100.f, 50.f, 50.f, cv::Size(100, 100));
			dv::camera::CameraGeometry b(100.f, 100.f, 50.f, 50.f, cv::Size(100, 100));
			dv::SparseEventBlockMatcher matcher(
				std::make_unique<dv::camera::StereoGeometry>(
					a, b, std::vector<float>({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1})),
				cv::Size(24, 24), 10, 20);
		}));
	};

	return EXIT_SUCCESS;
}

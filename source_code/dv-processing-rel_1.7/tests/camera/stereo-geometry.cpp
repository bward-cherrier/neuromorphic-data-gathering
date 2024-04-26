#include "../../include/dv-processing/camera/calibration_set.hpp"
#include "../../include/dv-processing/camera/stereo_geometry.hpp"
#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/depth/semi_dense_stereo_matcher.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/ccalib/randpattern.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>

int main(int ac, char **av) {
	using namespace boost::ut;
	namespace fs = std::filesystem;

	bool showPreview = false;

	CLI::App app{"Image feature tracker test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	std::string calibrationFile = "./calibration_files/stereo_calibration.json";
	auto calibration            = dv::camera::CalibrationSet::LoadFromFile(calibrationFile);
	auto geometry               = dv::camera::StereoGeometry(
        calibration.getCameraCalibration("C0").value(), calibration.getCameraCalibration("C1").value());

	cv::Size resolution      = calibration.getCameraCalibration("C0").value().resolution;
	cv::Size rightResolution = calibration.getCameraCalibration("C1").value().resolution;

	cv::randpattern::RandomPatternGenerator generator(resolution.width, resolution.height);
	generator.generatePattern();

	cv::Mat image = generator.getPattern();

	cv::Mat undistortedLeft  = geometry.remapImage(dv::camera::StereoGeometry::CameraPosition::Left, image);
	cv::Mat undistortedRight = geometry.remapImage(dv::camera::StereoGeometry::CameraPosition::Right, image);

	"disparity"_test = [&undistortedLeft, &undistortedRight, &showPreview] {
		dv::SemiDenseStereoMatcher matcher(undistortedLeft.size(), undistortedRight.size());

		cv::Mat disparity = matcher.compute(undistortedLeft, undistortedRight);
		if (showPreview) {
			cv::namedWindow("Disparity");
			cv::Mat disparityU8;
			disparity.convertTo(disparityU8, CV_8UC1);
			cv::imshow("Disparity", disparityU8);
		}
	};

	if (showPreview) {
		cv::Mat preview;
		cv::hconcat(image, undistortedLeft, preview);
		cv::hconcat(preview, undistortedRight, preview);
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
		cv::imshow("Preview", preview);
		cv::waitKey(0);
	}

	dv::EventStore data              = dv::data::generate::eventTestSet(0, resolution);
	dv::EventStore eventsLeftUndist  = geometry.remapEvents(dv::camera::StereoGeometry::CameraPosition::Left, data);
	dv::EventStore eventsRightUndist = geometry.remapEvents(
		dv::camera::StereoGeometry::CameraPosition::Right, dv::data::generate::eventTestSet(0, rightResolution));

	dv::EdgeMapAccumulator accumulator(resolution);
	accumulator.accept(data);
	dv::Frame original = accumulator.generateFrame();
	accumulator.accept(eventsLeftUndist);
	dv::Frame leftPreview = accumulator.generateFrame();
	accumulator.accept(eventsRightUndist);
	dv::Frame rightPreview = accumulator.generateFrame();

	if (showPreview) {
		cv::Mat preview;
		cv::hconcat(original.image, leftPreview.image, preview);
		cv::hconcat(preview, rightPreview.image, preview);
		cv::imshow("Preview", preview);
		cv::waitKey(0);
	}

	"out_of_bounds_events"_test = [&geometry, &resolution] {
		dv::EventStore store;
		store.emplace_back(0, 0, 0, true);
		store.emplace_back(0, 0, 0, false);
		store.emplace_back(0, resolution.width - 1, resolution.height - 1, true);
		store.emplace_back(0, resolution.width - 1, resolution.height - 1, false);
		store.emplace_back(0, resolution.width / 2, resolution.height / 2, true);
		dv::EventStore outOfBounds = geometry.remapEvents(dv::camera::StereoGeometry::CameraPosition::Left, store);
		expect(eq(outOfBounds.size(), 1));
	};

	"unmap_projection"_test = [&geometry] {
		namespace dvc = dv::camera;
		cv::Point2f coord(50.f, 50.f);
		auto undistortedRectified = geometry.remapPoint(dvc::StereoGeometry::CameraPosition::Left, coord);
		auto distortedUnrectified
			= geometry.unmapPoint<cv::Point2f>(dvc::StereoGeometry::CameraPosition::Left, undistortedRectified.value());
		expect(eq(distortedUnrectified.x, 50._f));
		expect(eq(distortedUnrectified.y, 50._f));
		undistortedRectified = geometry.remapPoint(dvc::StereoGeometry::CameraPosition::Right, coord);
		distortedUnrectified = geometry.unmapPoint<cv::Point2f>(
			dvc::StereoGeometry::CameraPosition::Right, undistortedRectified.value());
		expect(eq(distortedUnrectified.x, 50._f));
		expect(eq(distortedUnrectified.y, 50._f));
	};
}

//
// Created by rokas on 19.08.21.
// Copyright (c) 2021 iniVation AG. All rights reserved.
//

#include "../../include/dv-processing/camera/calibration_set.hpp"
#include "../../include/dv-processing/camera/camera_geometry.hpp"
#include "../../include/dv-processing/core/frame.hpp"

#include "../utilities/EventDatasetReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

#include <filesystem>

cv::Mat drawPoints(const cv::Size &resolution, const std::vector<cv::Point2f> &pixels);

dv::camera::CameraGeometry::UniquePtr loadFromTxt(const std::string &filename, const cv::Size &_resolution);

int main(int ac, char **av) {
	using namespace boost::ut;
	namespace fs = std::filesystem;

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

	"distortion_exceptions"_test = [] {
		cv::Size resolution(100, 100);

		// No distortion camera geometry
		dv::camera::CameraGeometry cam(static_cast<float>(resolution.width) / 2.f,
			static_cast<float>(resolution.height) / 2.f, static_cast<float>(resolution.width) / 2.f,
			static_cast<float>(resolution.height) / 2.f, resolution);

		expect(!cam.isUndistortionAvailable());

		expect(throws([&] {
			auto _ = cam.undistortEvents(dv::EventStore());
		}));

		expect(throws([&] {
			auto _ = cam.undistortSequence<std::vector<cv::Point2f>, std::vector<cv::Point2f>>({cv::Point2f(0.f, 0.f)});
		}));

		expect(throws([&] {
			auto _ = cam.backProjectUndistortSequence<std::vector<cv::Point3f>, std::vector<cv::Point2f>>(
				{cv::Point2f(0.f, 0.f)});
		}));

		expect(throws([&] {
			auto _
				= cam.distortSequence<std::vector<cv::Point3f>, std::vector<cv::Point3f>>({cv::Point3f(0.f, 0.f, 1.f)});
		}));
	};

	"within_dimensions"_test = [] {
		cv::Size resolution(100, 100);
		// No distortion camera geometry
		dv::camera::CameraGeometry cam(static_cast<float>(resolution.width) / 2.f,
			static_cast<float>(resolution.height) / 2.f, static_cast<float>(resolution.width) / 2.f,
			static_cast<float>(resolution.height) / 2.f, resolution);

		expect(cam.isWithinDimensions(dv::Point2f(0.f, 0.f)));
		expect(cam.isWithinDimensions(dv::Point2f(50.f, 50.f)));
		expect(cam.isWithinDimensions(dv::Point2f(99.f, 99.f)));
		expect(!cam.isWithinDimensions(dv::Point2f(99.5f, 99.5f)));
		expect(cam.isWithinDimensions(cv::Point2i(0, 0)));
		expect(!cam.isWithinDimensions(cv::Point2i(-1, -1)));
		expect(!cam.isWithinDimensions(cv::Point2i(100, 100)));
	};

	"reprojection"_test = [] {
		std::string calibrationFile = "./calibration_files/stereo_calibration.json";
		auto calibration            = dv::camera::CalibrationSet::LoadFromFile(calibrationFile);
		auto geometry               = calibration.getCameraCalibration("C0").value().getCameraGeometry();

		cv::Point2f point(50.f, 50.f);
		const auto forward   = geometry.backProjectUndistort<cv::Point3f>(point);
		const auto distorted = geometry.distort<cv::Point3f>(forward);
		const auto original  = geometry.project<cv::Point2f>(distorted);
		expect(eq(original.x, 50._f));
		expect(eq(original.y, 50._f));
	};

	"undistorting"_test = [] {
		dv::camera::CameraGeometry camera({-0.152558103f, -0.149609044f, 0.0120611433f, 0.020288324f}, 546.989258f,
			549.314514f, 281.37912f, 217.476562f, cv::Size(640, 480), dv::camera::DistortionModel::RadTan);

		auto undistorted = camera.undistort<cv::Point2f>(cv::Point2f(81.1f, 82.1f));

		expect(eq(undistorted.x, 64._f));
		expect(eq(undistorted.y, 70._f));

		dv::EventStore store;
		store.emplace_back(0, 81, 82, true);

		auto undistortedStore = camera.undistortEvents(store);
		expect(eq(undistortedStore.size(), 1));
		expect(eq(undistortedStore[0].x(), 64));
		expect(eq(undistortedStore[0].y(), 70));
	};

	if (!fs::exists(testDataset)) {
		return EXIT_SUCCESS;
	}

	test::EventDatasetReader reader(testDataset);

	auto image = reader.readNextImage();
	if (!image) {
		throw std::runtime_error("Image file is not available");
	}
	auto events = reader.readEventBatch(20000);
	if (!events) {
		throw std::runtime_error("Events in the dataset are not available");
	}

	cv::Size resolution = image->second.size();
	auto camera         = loadFromTxt(reader.getCalibrationPath(), resolution);

	"load_parameters"_test = [&] {
		expect(gt(camera->getFocalLength().x, 0.f));
		expect(gt(camera->getFocalLength().y, 0.f));
		expect(gt(camera->getCentralPoint().x, 0.f));
		expect(gt(camera->getCentralPoint().y, 0.f));
		expect(eq(camera->getDistortion().size(), 5ULL));
		expect(camera->isUndistortionAvailable());
	};

	"reproject"_test = [&] {
		cv::Point2f testPoint(50.f, 50.f);

		auto backproject = camera->backProject<cv::Point3f>(testPoint);
		boost::ut::expect(boost::ut::gt(std::abs(backproject.x - 0.f), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(boost::ut::gt(std::abs(backproject.y - 0.f), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(boost::ut::lt(std::abs(backproject.z - 1.f), std::numeric_limits<float>::epsilon()));

		auto projected = camera->project<cv::Point2f>(backproject);
		expect(lt(projected.x - testPoint.x, std::numeric_limits<float>::epsilon()));
		expect(lt(projected.y - testPoint.y, std::numeric_limits<float>::epsilon()));

		// Multiplying by a scalar (depth) should not affect projected coordinates in the image plane
		projected = camera->project<cv::Point2f>(backproject * 3.f);
		expect(lt(projected.x - testPoint.x, std::numeric_limits<float>::epsilon()));
		expect(lt(projected.y - testPoint.y, std::numeric_limits<float>::epsilon()));
	};

	"redistort"_test = [&] {
		cv::Point2f testPoint(50.f, 50.f);

		auto backproject = camera->backProjectUndistort<cv::Point3f>(testPoint);
		boost::ut::expect(boost::ut::gt(std::abs(backproject.x - 0.f), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(boost::ut::gt(std::abs(backproject.y - 0.f), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(boost::ut::lt(std::abs(backproject.z - 1.f), std::numeric_limits<float>::epsilon()));

		// The undistorted value should not be the same
		auto projected = camera->project<cv::Point2f>(backproject);
		boost::ut::expect(boost::ut::gt(std::abs(projected.y - testPoint.y), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(boost::ut::gt(std::abs(projected.y - testPoint.y), std::numeric_limits<float>::epsilon()));

		// Distorting back to an original camera frame, rounding tests it within 0.5px radius
		// since numerically it will be off be some small offset due to floating point
		// inaccuracies
		auto distorted = camera->project<cv::Point2f>(camera->distort<cv::Point3f>(backproject));
		boost::ut::expect(
			boost::ut::lt(std::abs(std::round(distorted.x) - testPoint.x), std::numeric_limits<float>::epsilon()));
		boost::ut::expect(
			boost::ut::lt(std::abs(std::round(distorted.y) - testPoint.y), std::numeric_limits<float>::epsilon()));
	};

	"reproject_batch"_test = [&] {
		std::vector<cv::Point2f> testPoints = {
			{10.f,  10.f },
            {20.f,  50.f },
            {150.f, 150.f}
        };
		auto backprojectedPoints = camera->backProjectSequence<std::vector<cv::Point3f>>(testPoints);
		expect(eq(backprojectedPoints.size(), testPoints.size()));

		auto reprojected = camera->projectSequence<dv::cvector<cv::Point2f>>(backprojectedPoints);
		expect(eq(reprojected.size(), testPoints.size()));

		size_t index = 0;
		for (const auto &point : reprojected) {
			expect(lt(std::abs(point.x - testPoints[index].x), 0.001f));
			expect(lt(std::abs(point.y - testPoints[index].y), 0.001f));
			index++;
		}
	};

	"redistort_batch"_test = [&] {
		std::vector<cv::Point2f> testPoints = {
			{10.f,  10.f },
            {20.f,  50.f },
            {150.f, 150.f}
        };

		auto backprojectedPoints = camera->backProjectUndistortSequence<std::vector<cv::Point3f>>(testPoints);
		for (const auto &point : backprojectedPoints) {
			expect(gt(std::abs(point.x - 0.f), std::numeric_limits<float>::epsilon()));
			expect(gt(std::abs(point.y - 0.f), std::numeric_limits<float>::epsilon()));
			expect(lt(std::abs(point.z - 1.f), std::numeric_limits<float>::epsilon()));
		}

		// Distorting back to an original camera frame, rounding tests it within 0.5px radius
		// since numerically it will be off be some small offset due to floating point
		// inaccuracies
		auto distortedPoints     = camera->distortSequence<std::vector<cv::Point3f>>(backprojectedPoints);
		auto distortedProjection = camera->projectSequence<std::vector<cv::Point2f>>(distortedPoints);

		size_t index = 0;
		for (const auto &point : distortedProjection) {
			expect(lt(std::abs(std::round(point.x) - testPoints[index].x), std::numeric_limits<float>::epsilon()));
			expect(lt(std::abs(std::round(point.y) - testPoints[index].y), std::numeric_limits<float>::epsilon()));
			index++;
		}
	};

	"undistort_events"_test = [&] {
		dv::EventStore testPoints;
		testPoints.emplace_back(0, 0, 0, true);
		testPoints.emplace_back(0, 239, 179, true);

		// These are extreme corner coordinates, they should not be there anymore
		auto undistorted = camera->undistortEvents(testPoints);
		expect(eq(undistorted.size(), 0ULL));

		dv::EventStore testPoints2;
		testPoints2.emplace_back(0, 100, 100, true);
		testPoints2.emplace_back(0, 120, 120, true);
		undistorted = camera->undistortEvents(testPoints2);
		expect(eq(undistorted.size(), 2ULL));
	};

	"camera_matrix"_test = [&] {
		cv::Matx33f cameraMatrix = camera->getCameraMatrix();
		auto focalLength         = camera->getFocalLength();
		auto centralPoint        = camera->getCentralPoint();

		expect(eq(cameraMatrix(0, 0), focalLength.x));
		expect(eq(cameraMatrix(0, 1), 0.f));
		expect(eq(cameraMatrix(0, 2), centralPoint.x));
		expect(eq(cameraMatrix(1, 0), 0.f));
		expect(eq(cameraMatrix(1, 1), focalLength.y));
		expect(eq(cameraMatrix(1, 2), centralPoint.y));
		expect(eq(cameraMatrix(2, 0), 0.f));
		expect(eq(cameraMatrix(2, 1), 0.f));
		expect(eq(cameraMatrix(2, 2), 1.f));
	};

	if (showPreview) {
		dv::Frame preview;

		// Convert to coordinates only
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
		std::vector<cv::Point2f> pixels;
		for (const auto &event : *events) {
			pixels.emplace_back(event.x(), event.y());
		}

		// Simple accumulation
		preview.image = drawPoints(resolution, pixels);
		cv::imshow("Preview", preview.image);
		cv::waitKey(0);

		// Undistortion and back projection
		auto undistorted           = camera->backProjectUndistortSequence<std::vector<cv::Point3f>>(pixels);
		auto undistortedProjection = camera->projectSequence<std::vector<cv::Point2f>>(undistorted);
		preview.image              = drawPoints(resolution, undistortedProjection);

		cv::imshow("Preview", preview.image);
		cv::waitKey(0);

		// This should distort back to the original image
		auto distorted           = camera->distortSequence<std::vector<cv::Point3f>>(undistorted);
		auto distortedProjection = camera->projectSequence<std::vector<cv::Point2f>>(distorted);
		preview.image            = drawPoints(resolution, distortedProjection);

		cv::imshow("Preview", preview.image);
		cv::waitKey(0);

		dv::EdgeMapAccumulator accumulator(resolution);
		accumulator.accumulate(*events);
		preview = accumulator.generateFrame();

		cv::imshow("Preview", preview.image);
		cv::waitKey(0);

		dv::EventStore undistoredEvents = camera->undistortEvents(*events);
		accumulator.accumulate(undistoredEvents);
		preview = accumulator.generateFrame();

		cv::imshow("Preview", preview.image);
		cv::waitKey(0);
	}

	return EXIT_SUCCESS;
}

cv::Mat drawPoints(const cv::Size &resolution, const std::vector<cv::Point2f> &pixels) {
	cv::Mat preview(resolution, CV_8UC1);
	preview           = 0;
	uint8_t increment = 255 / 4;
	uint8_t max       = increment * 4;
	for (const auto &pixel : pixels) {
		auto &value = preview.at<uint8_t>(pixel);
		value       = static_cast<uint8_t>(value + ((value >= max) ? 0 : increment));
	}
	return preview;
}

dv::camera::CameraGeometry::UniquePtr loadFromTxt(const std::string &filename, const cv::Size &_resolution) {
	std::ifstream calibFile(filename);
	float fx, fy, cx, cy, d1, d2, d3, d4, d5;

	if (calibFile >> fx >> fy >> cx >> cy >> d1 >> d2 >> d3 >> d4 >> d5) {
		return std::make_unique<dv::camera::CameraGeometry>(
			std::vector<float>({d1, d2, d3, d4, d5}), fx, fy, cx, cy, _resolution, dv::camera::DistortionModel::RadTan);
	}
	return nullptr;
}

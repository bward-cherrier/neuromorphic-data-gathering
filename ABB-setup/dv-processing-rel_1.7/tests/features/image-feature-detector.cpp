//
// Created by rokas on 02.08.21.
// Copyright (c) 2021 iniVation AG. All rights reserved.
//

#include "../../include/dv-processing/features/feature_detector.hpp"

#include "../utilities/EventDatasetReader.hpp"
#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>

cv::Mat generateImage(const cv::Size &resolution) {
	cv::Mat image = cv::Mat::zeros(resolution, CV_8UC1);
	cv::Point point(resolution.width / 10, resolution.height / 10);
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	point *= 2;
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	point += point;
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	return image;
}

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

	"no_pyr_detect"_test = [] {
		cv::Size resolution(100, 100);
		dv::features::ImageFeatureDetector detector(resolution, cv::GFTTDetector::create());

		cv::Mat image = generateImage(resolution);

		auto result = detector.runDetection(dv::Frame(0, image), 100);
		expect(gt(result.size(), 0));
	};

	"custom_type_detect"_test = [] {
		cv::Size resolution(100, 100);
		cv::Mat image = generateImage(resolution);

		struct MyCustomContainer {
			int64_t timestamp;
			cv::Mat image;
		};

		auto detector
			= dv::features::FeatureDetector<MyCustomContainer, cv::Feature2D>(resolution, cv::GFTTDetector::create());

		auto result = detector.runDetection(MyCustomContainer{0, image}, 100);
		expect(gt(result.size(), 0));
	};

	"image_pyramid_type_detect"_test = [] {
		cv::Size resolution(100, 100);
		cv::Mat image = generateImage(resolution);

		auto detector = dv::features::ImagePyrFeatureDetector(resolution, cv::GFTTDetector::create());

		auto result = detector.runDetection(dv::features::ImagePyramid(0, image), 100);
		expect(gt(result.size(), 0));
	};

	if (!fs::exists("samples/data")) {
		return EXIT_SUCCESS;
	}

	std::string image_path = "samples/data/home.jpg";
	cv::Mat img            = test::EventDatasetReader::imread(image_path, cv::IMREAD_COLOR);
	cv::Mat grayscale;
	cv::cvtColor(img, grayscale, cv::COLOR_BGR2GRAY);

	dv::features::ImagePyrFeatureDetector detector(img.size(), cv::FastFeatureDetector::create(10));
	auto corners = detector.runDetection(dv::features::ImagePyramid(0, grayscale), 100);

	expect(ge(corners.size(), 90ULL));
	expect(le(corners.size(), 110ULL));

	if (showPreview) {
		cv::drawKeypoints(img, dv::data::fromTimedKeyPoints(corners), img);
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
		cv::imshow("Preview", img);
		cv::waitKey(0);
	}

	return EXIT_SUCCESS;
}

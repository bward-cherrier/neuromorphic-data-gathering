//
// Created by rokas on 01.09.21.
//

#include "../../include/dv-processing/data/timed_keypoint_base.hpp"
#include "../../include/dv-processing/data/utilities.hpp"

#include "boost/ut.hpp"

dv::DepthFrame generateDepthFrame() {
	dv::DepthFrame depthFrame;
	depthFrame.sizeX     = 100;
	depthFrame.sizeY     = 100;
	depthFrame.minDepth  = 200;
	depthFrame.maxDepth  = 10000;
	depthFrame.timestamp = 10000000;
	depthFrame.step      = 5;
	depthFrame.depth.resize(static_cast<size_t>(depthFrame.sizeX * depthFrame.sizeY));
	depthFrame.depth.at(1) = depthFrame.minDepth;
	depthFrame.depth.at(2) = depthFrame.maxDepth;
	return depthFrame;
}

int main() {
	using namespace boost::ut;

	"basic_depth_mapping"_test = [] {
		dv::DepthFrame depthFrame = generateDepthFrame();
		cv::Mat depth             = dv::data::depthFrameMap(depthFrame);
		expect(eq(depth.channels(), 1));
		expect(eq(depth.type(), CV_16UC1));

		expect(eq(depth.at<uint16_t>(0, 0), 0));
		expect(eq(depth.at<uint16_t>(0, 1), depthFrame.minDepth));
		expect(eq(depth.at<uint16_t>(0, 2), depthFrame.maxDepth));
		expect(eq(depth.at<uint16_t>(0, 3), 0));
		expect(eq(depth.at<uint16_t>(1, 3), 0));
	};

	"depth_scaling"_test = [] {
		dv::DepthFrame depthFrame = generateDepthFrame();
		cv::Mat depth             = dv::data::depthFrameInMeters(depthFrame);
		expect(eq(depth.channels(), 1));
		expect(eq(depth.type(), CV_32FC1));

		float minDepth = static_cast<float>(depthFrame.minDepth) / 1000.f;
		float maxDepth = static_cast<float>(depthFrame.maxDepth) / 1000.f;

		expect(eq(depth.at<float>(0, 0), 0.f));
		expect(eq(depth.at<float>(0, 1), minDepth));
		expect(eq(depth.at<float>(0, 2), maxDepth));
		expect(eq(depth.at<float>(0, 3), 0.f));
		expect(eq(depth.at<float>(1, 3), 0.f));
	};

	"depth_conversion_from_opencv_integers"_test = [] {
		dv::DepthFrame depthFrame = generateDepthFrame();
		cv::Mat depth             = dv::data::depthFrameMap(depthFrame);
		dv::DepthFrame newFrame   = dv::data::depthFrameFromCvMat(depth);
		expect(eq(newFrame.depth.at(0), depthFrame.depth.at(0)));
		expect(eq(newFrame.depth.at(1), depthFrame.depth.at(1)));
		expect(eq(newFrame.depth.at(2), depthFrame.depth.at(2)));
		expect(eq(newFrame.depth.at(3), depthFrame.depth.at(3)));
	};

	"depth_conversion_from_opencv_floats"_test = [] {
		dv::DepthFrame depthFrame = generateDepthFrame();
		cv::Mat depth             = dv::data::depthFrameInMeters(depthFrame);
		dv::DepthFrame newFrame   = dv::data::depthFrameFromCvMat(depth);
		expect(eq(newFrame.depth.at(0), depthFrame.depth.at(0)));
		expect(eq(newFrame.depth.at(1), depthFrame.depth.at(1)));
		expect(eq(newFrame.depth.at(2), depthFrame.depth.at(2)));
		expect(eq(newFrame.depth.at(3), depthFrame.depth.at(3)));
	};

	return EXIT_SUCCESS;
}

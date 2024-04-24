#pragma once

#include "../data/frame_base.hpp"

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>

#include <memory>

namespace dv::features {

/**
 * Class that holds image pyramid layers with an according timestamp.
 */
class ImagePyramid {
public:
	typedef std::shared_ptr<ImagePyramid> SharedPtr;
	typedef std::unique_ptr<ImagePyramid> UniquePtr;

	/**
	 * Timestamp of the image pyramid.
	 */
	int64_t timestamp;

	/**
	 * Pyramid layers of the image.
	 */
	std::vector<cv::Mat> pyramid;

	/**
	 * Construct the image pyramid.
	 * @param timestamp_        Image timestamp.
	 * @param image             Image values.
	 * @param winSize           Window size for the search.
	 * @param maxPyrLevel       Maximum pyramid layer id (zero-based).
	 */
	ImagePyramid(int64_t timestamp_, const cv::Mat &image, const cv::Size &winSize, int maxPyrLevel) :
		timestamp(timestamp_) {
		cv::buildOpticalFlowPyramid(image, pyramid, winSize, maxPyrLevel);
	}

	/**
	 * Construct the image pyramid.
	 * @param frame             dv::Frame containing an image and timestamp.
	 * @param winSize           Window size for the search.
	 * @param maxPyrLevel       Maximum pyramid layer id (zero-based).
	 */
	ImagePyramid(const dv::Frame &frame, const cv::Size &winSize, int maxPyrLevel) : timestamp(frame.timestamp) {
		cv::buildOpticalFlowPyramid(frame.image, pyramid, winSize, maxPyrLevel);
	}

	/**
	 * Create a single layer image representation (no pyramid is going to be built).
	 * @param timestamp_        Image timestamp.
	 * @param image             Image values.
	 */
	ImagePyramid(int64_t timestamp_, const cv::Mat &image) : timestamp(timestamp_) {
		pyramid.push_back(image);
	}
};

} // namespace dv::features

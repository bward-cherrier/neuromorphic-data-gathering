#pragma once

#include "../core/core.hpp"
#include "../kinematics/transformation.hpp"
#include "depth_event_base.hpp"
#include "depth_frame_base.hpp"
#include "event_base.hpp"
#include "pose_base.hpp"
#include "timed_keypoint_base.hpp"

#include <opencv2/core.hpp>

namespace dv::data {

/**
 * Convert TimedKeyPoint vector into cv::KeyPoint vector.
 * @param points        KeyPoints to be converted.
 * @return              A vector of cv::KeyPoint.
 */
[[nodiscard]] inline std::vector<cv::KeyPoint> fromTimedKeyPoints(const dv::cvector<dv::TimedKeyPoint> &points) {
	std::vector<cv::KeyPoint> result;
	result.reserve(points.size());
	for (const auto &point : points) {
		result.emplace_back(
			point.pt.x(), point.pt.y(), point.size, point.angle, point.response, point.octave, point.class_id);
	}
	return result;
}

/**
 * Convert TimedKeyPoint vector into cv::Point2f vector.
 * @param points        KeyPoints to be converted.
 * @return              A vector of cv::Point2f.
 */
[[nodiscard]] inline std::vector<cv::Point2f> convertToCvPoints(const dv::cvector<dv::TimedKeyPoint> &points) {
	std::vector<cv::Point2f> result;
	result.reserve(points.size());
	for (const auto &point : points) {
		result.emplace_back(point.pt.x(), point.pt.y());
	}
	return result;
}

/**
 * Create a vector of `cv::KeyPoint` from a given vector of `dv::TimedKeyPoint`.
 * @param points        cv::KeyPoint vector to be converted.
 * @param defaultTime   Timestamp in microseconds to be assigned to all new TimedKeyPoints.
 * @return              A vector of TimedKeyPoints.
 */
[[nodiscard]] inline dv::cvector<dv::TimedKeyPoint> fromCvKeypoints(
	const std::vector<cv::KeyPoint> &points, const int64_t defaultTime = 0) {
	dv::cvector<dv::TimedKeyPoint> result;
	result.reserve(result.size());
	for (const auto &point : points) {
		result.emplace_back(dv::Point2f(point.pt.x, point.pt.y), point.size, point.angle, point.response, point.octave,
			point.class_id, defaultTime);
	}
	return result;
}

/**
 * Map a depth frame into an OpenCV Mat, no data copies are performed. The resulting cv::Mat
 * will point to the same underlying data.
 *
 * This function does not affect any data underlying, the const qualifier is not set since
 * cv::Mat can't be const.
 * @param frame 		Frame to be mapped.
 * @return 				Mapped depth frame in `cv::Mat` with data type of `CV_16UC1`.
 */
[[nodiscard]] inline cv::Mat depthFrameMap(dv::DepthFrame &frame) {
	return {static_cast<int>(frame.sizeX), static_cast<int>(frame.sizeY), CV_16UC1,
		reinterpret_cast<void *>(frame.depth.data()), static_cast<size_t>(frame.sizeX) * sizeof(uint16_t)};
}

/**
 * Converts the given depth frame into an OpenCV matrix containing depth values in meters.
 *
 * Resulting `cv::Mat` will be of floating type and will apply conversion from millimeters
 * to meters. Depth value of 0.0f should be considered invalid.
 *
 * This function will copy and scale all values into meters.
 * @param frame		Depth frame to be converted.
 * @return			A `cv::Mat` containing scaled depth values in meters.
 */
[[nodiscard]] inline cv::Mat depthFrameInMeters(dv::DepthFrame &frame) {
	cv::Mat raw = depthFrameMap(frame);
	cv::Mat depthInMeters;
	raw.convertTo(depthInMeters, CV_32FC1, 1e-3);
	return depthInMeters;
}

/**
 * Converts the given OpenCV matrix with depth values to DepthFrame.
 *
 * `cv::Mat` can contain single-channel floating point containing depth values in meters or
 * single-channel 16-bit unsigned integer values in millimeters. Zero should be used for
 * invalid values.
 *
 * This function will copy and scale all values into millimeter 16-bit integer representation.
 * @param depthImage 	`cv::Mat` containing the depth values.
 * @return 				Depth frame containing depth values in 16-bit unsigned integer values
 * 						representing distance in millimeters.
 */
[[nodiscard]] inline dv::DepthFrame depthFrameFromCvMat(const cv::Mat &depthImage) {
	if (!(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1)) {
		throw std::invalid_argument(
			"Input image type is not supported; Please pass depth image containing depth values in floating point "
			"meter representation or unsigned 16-bit integer containing millimeters.");
	}
	dv::DepthFrame depthFrame;
	depthFrame.sizeX = static_cast<int16_t>(depthImage.cols);
	depthFrame.sizeY = static_cast<int16_t>(depthImage.rows);
	depthFrame.depth.resize(static_cast<size_t>(depthFrame.sizeX * depthFrame.sizeY));

	cv::Mat raw = depthFrameMap(depthFrame);
	if (depthImage.type() == CV_16UC1) {
		depthImage.copyTo(raw);
	}
	else {
		depthImage.convertTo(raw, CV_16UC1, 1e+3);
	}
	return depthFrame;
}

/**
 * Convert a pose message into a transformation.
 * @param pose 		Input pose to be converted.
 * @return 			Transformation representing the pose.
 */
template<std::floating_point Scalar = float>
[[nodiscard]] inline dv::kinematics::Transformation<Scalar> transformFromPose(const dv::Pose &pose) {
	return dv::kinematics::Transformation<Scalar>(pose.timestamp,
		Eigen::Matrix<Scalar, 3, 1>(pose.translation.x(), pose.translation.y(), pose.translation.z()),
		Eigen::Quaternion<Scalar>(pose.rotation.w(), pose.rotation.x(), pose.rotation.y(), pose.rotation.z()));
}

/**
 * Convert a transformation into a pose message.
 * @param transform		Input transform.
 * @return				Pose message representing the transform.
 */
template<std::floating_point Scalar = float>
[[nodiscard]] inline dv::Pose poseFromTransformation(const dv::kinematics::Transformation<Scalar> &transform) {
	const auto &translation = transform.getTranslation();
	const auto &quat        = transform.getQuaternion();
	return dv::Pose(transform.getTimestamp(), dv::Vec3f(translation(0), translation(1), translation(2)),
		dv::Quaternion(quat.w(), quat.x(), quat.y(), quat.z()), dv::cstring(), dv::cstring());
}

} // namespace dv::data

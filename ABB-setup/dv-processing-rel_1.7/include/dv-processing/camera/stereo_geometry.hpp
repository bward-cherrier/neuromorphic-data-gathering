#pragma once

#include "../core/utils.hpp"
#include "../data/depth_frame_base.hpp"
#include "calibrations/camera_calibration.hpp"
#include "camera_geometry.hpp"

#include <opencv2/imgproc.hpp>

namespace dv::camera {

/**
 * A class that performs stereo geometry operations and rectification of a stereo camera.
 */
class StereoGeometry {
public:
	/**
	 * Position enum for a single camera in a stereo configuration.
	 */
	enum class CameraPosition {
		Left,
		Right
	};
	enum class FunctionImplementation {
		LUT,
		SubPixel
	};

private:
	cv::Mat mLeftRemap1;
	cv::Mat mLeftRemap2;
	cv::Mat mRightRemap1;
	cv::Mat mRightRemap2;
	cv::Mat mLeftProjection;
	cv::Mat mRightProjection;

	std::vector<uint8_t> mLeftValidMask;
	std::vector<uint8_t> mRightValidMask;

	std::vector<cv::Point2i> mLeftRemapLUT;
	std::vector<cv::Point2i> mRightRemapLUT;
	std::vector<cv::Point2i> mLeftUnmapLUT;
	std::vector<cv::Point2i> mRightUnmapLUT;

	cv::Size mLeftResolution;
	cv::Size mRightResolution;

	std::vector<float> mDistLeft;
	DistortionModel mLeftDistModel;
	std::vector<float> mDistRight;
	DistortionModel mRightDistModel;

	cv::Mat RN[2], Q;
	dv::kinematics::Transformationf mLeftRectifierInverse;
	dv::kinematics::Transformationf mRightRectifierInverse;

	const dv::camera::CameraGeometry mOriginalLeft;
	const dv::camera::CameraGeometry mOriginalRight;

	float mBaseline;

	template<dv::concepts::Coordinate2DCostructible PointType = cv::Point2f>
	static std::vector<PointType> initCoordinateList(const cv::Size &resolution) {
		std::vector<PointType> coordinates;
		// Populate undistort events input map with all possible (x, y) address combinations.
		for (int32_t y = 0; y < resolution.height; y++) {
			for (int32_t x = 0; x < resolution.width; x++) {
				// Use center of pixel to get better approximation, since we're using floats anyway.
				coordinates.emplace_back(static_cast<float>(x), static_cast<float>(y));
			}
		}
		return coordinates;
	}

	static dv::EventStore remapEventsInternal(const dv::EventStore &events, const cv::Size &resolution,
		const std::vector<uint8_t> &mask, const std::vector<cv::Point2i> &remapLUT) {
		dv::EventStore output;
		for (const auto &event : events) {
			const auto pos = static_cast<size_t>(event.y()) * static_cast<size_t>(resolution.width)
						   + static_cast<size_t>(event.x());
			dv::runtime_assert(pos < mask.size(), "Event coordinates are out of range");
			if (mask[pos] == 1) {
				const cv::Point2i &coords = remapLUT[pos];
				output.emplace_back(event.timestamp(), coords.x, coords.y, event.polarity());
			}
		}
		return output;
	}

	void createLUTs(const cv::Size &resolution, const cv::Matx33f &cameraMatrix, const cv::Mat &distortion,
		const cv::Mat &R, const cv::Mat &P, std::vector<uint8_t> &outputMask,
		std::vector<cv::Point2i> &outputRemapLUT) const {
		std::vector<cv::Point2f> undistortEventOutputMap;
		const std::vector<cv::Point2f> undistortEventInputMap = initCoordinateList(resolution);
		cv::undistortPoints(undistortEventInputMap, undistortEventOutputMap, cameraMatrix, distortion, R, P);
		std::copy(undistortEventOutputMap.begin(), undistortEventOutputMap.end(), std::back_inserter(outputRemapLUT));

		for (size_t i = 0; i < outputRemapLUT.size(); ++i) {
			const cv::Point2i &coord = outputRemapLUT[i];
			if (coord.x < 0 || coord.y < 0 || coord.x >= resolution.width || coord.y >= resolution.height) {
				outputMask[i] = 0;
			}
		}
	}

	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate2D Input>
	[[nodiscard]] Output backProject(const StereoGeometry::CameraPosition position, const Input &pixel) const {
		double Fx;
		double Fy;
		double Cx;
		double Cy;

		switch (position) {
			case CameraPosition::Left:
				Fx = mLeftProjection.at<double>(0, 0);
				Fy = mLeftProjection.at<double>(1, 1);
				Cx = mLeftProjection.at<double>(0, 2);
				Cy = mLeftProjection.at<double>(1, 2);
				break;
			case CameraPosition::Right:
				Fx = mRightProjection.at<double>(0, 0);
				Fy = mRightProjection.at<double>(1, 1);
				Cx = mRightProjection.at<double>(0, 2);
				Cy = mRightProjection.at<double>(1, 2);
				break;
		}

		if constexpr (concepts::Coordinate2DMembers<Input>) {
			return Output((static_cast<double>(pixel.x) - Cx) / Fx, (static_cast<double>(pixel.y) - Cy) / Fy, 1);
		}
		else if (concepts::Coordinate2DAccessors<Input>) {
			return Output((static_cast<double>(pixel.x()) - Cx) / Fx, (static_cast<double>(pixel.y()) - Cy) / Fy, 1);
		}
	}

public:
	using UniquePtr = std::unique_ptr<StereoGeometry>;
	using SharedPtr = std::shared_ptr<StereoGeometry>;

	/**
	 * Initialize a stereo geometry class using two camera geometries for each of the stereo camera pair and a
	 * transformation matrix that describes the transformation from right camera to the left.
	 * @param leftCamera			Left camera geometry.
	 * @param rightCamera			Right camera geometry.
	 * @param transformToLeft		A vector containing a homogenous transformation from right to the left camera.
	 * 								Vector should contain exactly 16 numbers (as per 4x4 homogenous transformation
	 * matrix) in a row-major ordering.
	 * @param rectifiedResolution 	Resolution of the rectified image plane when remapping events/points/images
	 * 								from either the left or right camera (see remapEvents()/remapImage()).
	 * 								This can be the same, smaller, or larger than either the left or right camera
	 * 								resolutions, where downsampling/upsampling occurs if the #rectifiedResolution is
	 * 								smaller/larger than the camera resolution. Defaults to the left camera resolution if
	 * 								not provided.
	 */
	StereoGeometry(const CameraGeometry &leftCamera, const CameraGeometry &rightCamera,
		const std::vector<float> &transformToLeft, std::optional<cv::Size> rectifiedResolution = std::nullopt) :
		mLeftResolution(leftCamera.getResolution()),
		mRightResolution(rightCamera.getResolution()),
		mLeftDistModel(leftCamera.getDistortionModel()),
		mRightDistModel(rightCamera.getDistortionModel()),
		mOriginalLeft(leftCamera),
		mOriginalRight(rightCamera) {
		const cv::Mat transform
			= cv::Mat(cv::Size(4, 4), CV_32FC1, const_cast<void *>(static_cast<const void *>(transformToLeft.data())));

		cv::Mat R = transform(cv::Rect(0, 0, 3, 3));
		cv::Mat T = transform(cv::Rect(3, 0, 1, 3));

		mBaseline = T.at<float>(0, 0);

		const cv::Mat camLeft(leftCamera.getCameraMatrix());
		R.convertTo(R, CV_64FC1);
		T.convertTo(T, CV_64FC1);

		const cv::Mat camRight(rightCamera.getCameraMatrix());
		mDistLeft  = leftCamera.getDistortion();
		mDistRight = rightCamera.getDistortion();

		const cv::Mat distLeftMat(mDistLeft.size(), 1, CV_32FC1, mDistLeft.data());
		const cv::Mat distRightMat(mDistRight.size(), 1, CV_32FC1, mDistRight.data());

		// If new resolution argument was not passed, assign left resolution
		if (!rectifiedResolution.has_value()) {
			rectifiedResolution = mLeftResolution;
		}

		cv::stereoRectify(camLeft, distLeftMat, camRight, distRightMat, mLeftResolution, R, T, RN[0], RN[1],
			mLeftProjection, mRightProjection, Q, cv::CALIB_ZERO_DISPARITY, 0.0, rectifiedResolution.value());

		cv::initUndistortRectifyMap(camLeft, distLeftMat, RN[0], mLeftProjection, rectifiedResolution.value(), CV_16SC2,
			mLeftRemap1, mLeftRemap2);
		cv::initUndistortRectifyMap(camRight, distRightMat, RN[1], mRightProjection, rectifiedResolution.value(),
			CV_16SC2, mRightRemap1, mRightRemap2);

		mLeftValidMask.resize(static_cast<size_t>(mLeftResolution.area()), 1);
		mRightValidMask.resize(static_cast<size_t>(mRightResolution.area()), 1);

		mLeftRectifierInverse  = dv::kinematics::Transformationf(0, cv::Mat::zeros(3, 1, CV_32FC1), RN[0].inv());
		mRightRectifierInverse = dv::kinematics::Transformationf(0, cv::Mat::zeros(3, 1, CV_32FC1), RN[1].inv());

		createLUTs(mLeftResolution, camLeft, distLeftMat, RN[0], mLeftProjection, mLeftValidMask, mLeftRemapLUT);
		createLUTs(mRightResolution, camRight, distRightMat, RN[1], mRightProjection, mRightValidMask, mRightRemapLUT);

		for (const cv::Point2i &pixel : initCoordinateList<cv::Point2i>(mLeftResolution)) {
			auto p = unmapPoint<cv::Point2f, FunctionImplementation::SubPixel>(CameraPosition::Left, pixel);
			mLeftUnmapLUT.emplace_back(std::round(p.x), std::round(p.y));
		}
		for (const cv::Point2i &pixel : initCoordinateList<cv::Point2i>(mRightResolution)) {
			auto p = unmapPoint<cv::Point2f, FunctionImplementation::SubPixel>(CameraPosition::Right, pixel);
			mRightUnmapLUT.emplace_back(std::round(p.x), std::round(p.y));
		}
	}

	/**
	 * Create a stereo geometry class from left and right camera calibration instances.
	 * @param leftCalibration 	Left camera calibration.
	 * @param rightCalibration	Right camera calibration.
	 * @param rectifiedResolution 	Resolution of the rectified image plane when remapping events/points/images
	 * from either the left or right camera (see above constructor).
	 */
	StereoGeometry(const calibrations::CameraCalibration &leftCalibration,
		const calibrations::CameraCalibration &rightCalibration,
		std::optional<cv::Size> rectifiedResolution = std::nullopt) :
		StereoGeometry(leftCalibration.getCameraGeometry(), rightCalibration.getCameraGeometry(),
			computeTransformBetween(rightCalibration, leftCalibration), rectifiedResolution) {
	}

	/**
	 * Compute the homogeneous transformation that transforms a point from a source camera to a target camera based
	 * on their respective calibrations.
	 * @param src Camera calibration for the source camera.
	 * @param target Camera calibration for the target camera.
	 * @return 4x4 transformation from source to target.
	 */
	[[nodiscard]] static std::vector<float> computeTransformBetween(
		const calibrations::CameraCalibration &src, const calibrations::CameraCalibration &target) {
		const auto T_C0S                        = src.getTransformMatrix();
		const auto T_C0T                        = target.getTransformMatrix();
		const Eigen::Matrix4f transformToTarget = T_C0T.inverse() * T_C0S;

		const auto matrixView = transformToTarget.reshaped<Eigen::RowMajor>();
		return {matrixView.cbegin(), matrixView.cend()};
	}

	/**
	 * Apply remapping to an input image to rectify it.
	 * @param cameraPosition 	Indication whether image is from left or right camera.
	 * @param image 			Input image.
	 * @return 					Rectified image.
	 */
	[[nodiscard]] cv::Mat remapImage(const CameraPosition cameraPosition, const cv::Mat &image) const {
		cv::Mat output;
		switch (cameraPosition) {
			case CameraPosition::Left:
				cv::remap(image, output, mLeftRemap1, mLeftRemap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
				break;
			case CameraPosition::Right:
				cv::remap(image, output, mRightRemap1, mRightRemap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
				break;
			default:
				throw dv::exceptions::RuntimeError("Invalid camera position value");
		}
		return output;
	}

	/**
	 * Apply remapping on input events.
	 * @param cameraPosition 	Indication whether image is from left or right camera.
	 * @param events 			Input events.
	 * @return 					Event with rectified coordinates.
	 */
	[[nodiscard]] dv::EventStore remapEvents(const CameraPosition cameraPosition, const dv::EventStore &events) const {
		switch (cameraPosition) {
			case CameraPosition::Left:
				return remapEventsInternal(events, mLeftResolution, mLeftValidMask, mLeftRemapLUT);
			case CameraPosition::Right:
				return remapEventsInternal(events, mRightResolution, mRightValidMask, mRightRemapLUT);
			default:
				throw dv::exceptions::RuntimeError("Invalid camera position value");
		}
	}

	/**
	 * Remap a point coordinate from original camera pixel space into undistorted and rectified pixel space.
	 * @param cameraPosition 	Camera position in the stereo setup.
	 * @param point 			Coordinates in original camera pixel space.
	 * @return 					Undistorted and rectified coordinates or `std::nullopt` if the resulting coordinates
	 * 							are outside of valid output pixel range.
	 * @tparam Point
	 */
	template<dv::concepts::Coordinate2DCostructible OutputPoint = cv::Point2i, dv::concepts::Coordinate2D InputPoint>
	[[nodiscard]] std::optional<OutputPoint> remapPoint(
		const CameraPosition cameraPosition, const InputPoint &point) const {
		switch (cameraPosition) {
			case CameraPosition::Left: {
				size_t pos;
				if constexpr (dv::concepts::Coordinate2DMembers<InputPoint>) {
					pos = static_cast<size_t>(point.y) * static_cast<size_t>(mLeftResolution.width)
						+ static_cast<size_t>(point.x);
				}
				else {
					pos = static_cast<size_t>(point.y()) * static_cast<size_t>(mLeftResolution.width)
						+ static_cast<size_t>(point.x());
				}
				dv::runtime_assert(pos < mLeftValidMask.size(), "Event coordinates are out of range");
				if (mLeftValidMask[pos] == 1) {
					const cv::Point2i &coords = mLeftRemapLUT[pos];
					return OutputPoint(coords.x, coords.y);
				}
				return std::nullopt;
			}
			case CameraPosition::Right: {
				size_t pos;
				if constexpr (dv::concepts::Coordinate2DMembers<InputPoint>) {
					pos = static_cast<size_t>(point.y) * static_cast<size_t>(mRightResolution.width)
						+ static_cast<size_t>(point.x);
				}
				else {
					pos = static_cast<size_t>(point.y()) * static_cast<size_t>(mRightResolution.width)
						+ static_cast<size_t>(point.x());
				}
				dv::runtime_assert(pos < mRightValidMask.size(), "Event coordinates are out of range");
				if (mRightValidMask[pos] == 1) {
					const cv::Point2i &coords = mRightRemapLUT[pos];
					return OutputPoint(coords.x, coords.y);
				}
				return std::nullopt;
			}
			default:
				throw dv::exceptions::RuntimeError("Invalid camera position argument value");
		}
	}

	/**
	 * Unmap a point coordinate from undistorted and rectified pixel space into original distorted pixel.
	 * @param position 			Camera position in the stereo setup
	 * @param point 			Coordinates in undistorted rectified pixel space.
	 * @return 					Coordinates of the pixel in original pixel space.
	 * @tparam OutputPoint 		Output point class
	 * @tparam Implementation	Implementation type: LUT - performs a look-up operation on a precomputed look-up table,
	 * 							SubPixel - performs full computations and retrieves exact coordinates.
	 * @tparam InputPoint		Input point class (automatically inferred)
	 */
	template<dv::concepts::Coordinate2DCostructible OutputPoint = cv::Point2i,
		FunctionImplementation Implementation = FunctionImplementation::LUT, dv::concepts::Coordinate2D InputPoint>
	[[nodiscard]] OutputPoint unmapPoint(const CameraPosition position, const InputPoint &point) const {
		if constexpr (Implementation == FunctionImplementation::LUT) {
			const auto &resolution = (position == CameraPosition::Left ? mLeftResolution : mRightResolution);
			dv::runtime_assert(
				dv::isWithinDimensions(point, resolution), "Coordinate for unmapping is outside of valid dimensions");
			size_t address;
			auto width = static_cast<const size_t>(resolution.width);
			if constexpr (concepts::Coordinate2DMembers<InputPoint>) {
				address = static_cast<size_t>(point.y) * width + static_cast<size_t>(point.x);
			}
			else if constexpr (concepts::Coordinate2DAccessors<InputPoint>) {
				address = static_cast<size_t>(point.y()) * width + static_cast<size_t>(point.x());
			}
			switch (position) {
				case CameraPosition::Left: {
					const cv::Point2i &p = mLeftUnmapLUT[address];
					return OutputPoint(p.x, p.y);
				}
				case CameraPosition::Right: {
					const cv::Point2i &p = mRightUnmapLUT[address];
					return OutputPoint(p.x, p.y);
				}
				default:
					throw dv::exceptions::RuntimeError("Invalid camera position value");
			}
		}
		else if constexpr (Implementation == FunctionImplementation::SubPixel) {
			const auto backProjected = backProject<cv::Point3f>(position, point);
			switch (position) {
				case CameraPosition::Left: {
					auto unrectified = mLeftRectifierInverse.rotatePoint<cv::Point3f>(backProjected);
					if (mOriginalLeft.isUndistortionAvailable()) {
						unrectified = mOriginalLeft.distort<cv::Point3f>(unrectified);
					}
					return mOriginalLeft.project<OutputPoint>(unrectified);
				}
				case CameraPosition::Right: {
					auto unrectified = mRightRectifierInverse.rotatePoint<cv::Point3f>(backProjected);
					if (mOriginalRight.isUndistortionAvailable()) {
						unrectified = mOriginalRight.distort<cv::Point3f>(unrectified);
					}
					return mOriginalRight.project<OutputPoint>(unrectified);
				}
				default:
					throw dv::exceptions::RuntimeError("Invalid camera position value");
			}
		}
	}

	/**
	 * Retrieve left camera geometry class that can project coordinates into stereo rectified space.
	 * @return 		Camera geometry instance.
	 */
	[[nodiscard]] dv::camera::CameraGeometry getLeftCameraGeometry() const {
		return {mDistLeft, static_cast<float>(mLeftProjection.at<double>(0, 0)),
			static_cast<float>(mLeftProjection.at<double>(1, 1)), static_cast<float>(mLeftProjection.at<double>(0, 2)),
			static_cast<float>(mLeftProjection.at<double>(1, 2)), mLeftResolution, mLeftDistModel};
	}

	/**
	 * Retrieve right camera geometry class that can project coordinates into stereo rectified space.
	 * @return 		Camera geometry instance.
	 */
	[[nodiscard]] dv::camera::CameraGeometry getRightCameraGeometry() const {
		return {mDistRight, static_cast<float>(mRightProjection.at<double>(0, 0)),
			static_cast<float>(mRightProjection.at<double>(1, 1)),
			static_cast<float>(mRightProjection.at<double>(0, 2)),
			static_cast<float>(mRightProjection.at<double>(1, 2)), mRightResolution, mRightDistModel};
	}

	/**
	 * Estimate depth given the disparity map and a list of events. The coordinates will be rectified and
	 * a disparity value will be looked up in the disparity map. The depth of each event is calculated
	 * using an equation: depth = (focalLength * baseline) / disparity. Focal length is expressed
	 * in meter distance.
	 *
	 * For practical applications, depth estimation should be evaluated prior to any use. The directly estimated depth
	 * values can contain measurable errors which should be accounted for - the errors can usually be within 10-20%
	 * fixed absolute error distance. Usually this comes from various inaccuracies and can be mitigated by introducing
	 * a correction factor for the depth estimate.
	 * @param disparity			Disparity map.
	 * @param events			Input events.
	 * @param disparityScale	Scale of disparity value in the disparity map, if subpixel accuracy is enabled
	 * 							in the block matching, this value will be equal to 16.
	 * @return					A depth event store, the events will contain the same information as in the
	 * 							input, but additionally will have the depth value. Events whose coordinates
	 * 							are outside of image bounds after rectification will be skipped.
	 */
	[[nodiscard]] dv::DepthEventStore estimateDepth(
		const cv::Mat &disparity, const dv::EventStore &events, const float disparityScale = 16.f) const {
		DepthEventStore output;

		const float focalLength = static_cast<float>(mLeftProjection.at<double>(0, 0));

		const float baselineFocal = std::abs(focalLength * mBaseline);
		for (const auto &event : events) {
			const auto pos = static_cast<const size_t>(event.y()) * static_cast<size_t>(mLeftResolution.width)
						   + static_cast<size_t>(event.x());
			if (mLeftValidMask[pos]) {
				const cv::Point2i &pt      = mLeftRemapLUT[pos];
				const int16_t rawDisparity = disparity.at<int16_t>(pt);
				if (rawDisparity <= 0) {
					continue;
				}
				const float disp  = (static_cast<float>(rawDisparity) / disparityScale);
				const float depth = baselineFocal / disp;
				output.emplace_back(event.timestamp(), event.x(), event.y(), event.polarity(),
					cv::saturate_cast<uint16_t>(depth * 1000.f));
			}
		}

		return output;
	}

	/**
	 * Convert a disparity map into a depth frame. Each disparity value is converted into depth using the equation
	 * depth = (focalLength * baseline) / disparity. Output frame contains distance values expressed
	 * in integer values of millimeter distance.
	 *
	 * NOTE: Output depth frame will not have a timestamp value, it is up to the user of this method to set
	 * correct timestamp of the disparity map.
	 * @param disparity 		Input disparity map.
	 * @param disparityScale	Scale of disparity value in the disparity map, if subpixel accuracy is enabled
	 * 						in the block matching, this value will be equal to 16.
	 * @return 					A converted depth frame.
	 */
	[[nodiscard]] dv::DepthFrame toDepthFrame(const cv::Mat &disparity, const float disparityScale = 16.f) const {
		dv::DepthFrame dFrame;
		dFrame.sizeX = static_cast<int16_t>(disparity.cols);
		dFrame.sizeY = static_cast<int16_t>(disparity.rows);
		dFrame.depth.resize(static_cast<size_t>(dFrame.sizeX * dFrame.sizeY));

		cv::Mat depthFrame(dFrame.sizeY, dFrame.sizeX, CV_16UC1, dFrame.depth.data());

		cv::Mat floatDisparity;
		disparity.convertTo(floatDisparity, CV_32FC1, 1.0 / static_cast<double>(disparityScale));

		const double focalLength   = mLeftProjection.at<double>(0, 0);
		const double baselineFocal = std::abs(focalLength * static_cast<double>(mBaseline));

		floatDisparity = (baselineFocal / (floatDisparity));
		floatDisparity.convertTo(depthFrame, CV_16UC1, 1000.);

		dv::runtime_assert(dFrame.depth.size() == static_cast<size_t>(disparity.size().area()),
			"Output depth frame size doesn't match input disparity map size");

		return dFrame;
	}
};

} // namespace dv::camera

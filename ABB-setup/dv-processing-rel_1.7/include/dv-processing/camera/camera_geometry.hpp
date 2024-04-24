#pragma once

#include "../core/core.hpp"
#include "../data/timed_keypoint_base.hpp"
#include "../exception/exception.hpp"
#include "../kinematics/linear_transformer.hpp"

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <vector>

namespace dv::camera {

enum DistortionModel {
	None,
	RadTan,
	Equidistant
};

namespace internal {

static constexpr std::string_view NoneModelString{"none"};
static constexpr std::string_view RadialTangentialModelString{"radialTangential"};
static constexpr std::string_view EquidistantModelString{"equidistant"};

} // namespace internal

/**
 * Convert a string into the Enum of the DistortionModel
 * @param model
 * @return the enum corresponding to the string
 */
static DistortionModel stringToDistortionModel(const std::string_view model) {
	if (model == internal::RadialTangentialModelString) {
		return DistortionModel::RadTan;
	}
	else if (model == internal::EquidistantModelString) {
		return DistortionModel::Equidistant;
	}
	else if (model == internal::NoneModelString) {
		return DistortionModel::None;
	}
	else {
		throw dv::exceptions::InvalidArgument<std::string>(
			"Unsupported distortion model, currently supported models are ['radialTangential', 'equidistant', 'none']",
			std::string{model});
	}
}

/**
 * Convert a DistortionModel Enum into a string
 * @param model
 * @return the string that represent the Distortion model
 */
static std::string distortionModelToString(const DistortionModel &model) {
	if (model == DistortionModel::RadTan) {
		return std::string(internal::RadialTangentialModelString);
	}
	else if (model == DistortionModel::Equidistant) {
		return std::string(internal::EquidistantModelString);
	}
	else if (model == DistortionModel::None) {
		return std::string(internal::NoneModelString);
	}
	else {
		throw dv::exceptions::InvalidArgument<int>(
			"Unsupported distortion model, currently supported models are ['radialTangential', 'equidistant', 'none']",
			dv::EnumAsInteger(model));
	}
}

class CameraGeometry {
private:
	/**
	 * Row-based distortion look-up table. Access index by:
	 * index = (y * width) + x
	 */
	std::vector<cv::Point3f> mDistortionLUT;

	/**
	 * Row-based distortion look-up table. Access index by:
	 * index = (y * width) + x
	 */
	std::vector<cv::Point3f> mBackProjectLUT;

	/**
	 * Row-based undistorted coordinate look-up table, containing undistorted points in pixel space.
	 * Access index by:
	 * index = (y * width) + x
	 */
	std::vector<cv::Point2f> mDistortionPixelLUT;

	/**
	 * Generates internal distortion look-up table to speed up undistortion.
	 */
	void generateLUTs() {
		std::vector<cv::Point2f> allPixels;
		for (int y = 0.f; y < mResolution.height; y++) {
			for (int x = 0.f; x < mResolution.width; x++) {
				allPixels.emplace_back(static_cast<float>(x), static_cast<float>(y));
				mBackProjectLUT.push_back(
					backProject<cv::Point3f, cv::Point2f, FunctionImplementation::SubPixel>(allPixels.back()));
			}
		}
		if (!mDistortion.empty()) {
			std::vector<cv::Point2f> undistortedPixels;
			switch (mDistortionModel) {
				case DistortionModel::RadTan: {
					cv::undistortPoints(allPixels, undistortedPixels, getCameraMatrix(), mDistortion);
					for (const auto &p : undistortedPixels) {
						mDistortionLUT.emplace_back(p.x, p.y, 1.f);
					}
					break;
				}
				case DistortionModel::Equidistant: {
					cv::fisheye::undistortPoints(allPixels, undistortedPixels, getCameraMatrix(), mDistortion);
					for (const auto &p : undistortedPixels) {
						mDistortionLUT.emplace_back(p.x, p.y, 1.f);
					}
					break;
				}
				case DistortionModel::None: {
					break;
				}
				default:
					throw dv::exceptions::InvalidArgument<int>(
						"Invalid distortion model", dv::EnumAsInteger(mDistortionModel));
			}

			mDistortionPixelLUT = projectSequence<std::vector<cv::Point2f>>(mDistortionLUT, false);
		}
	}

	/**
	 * Distortion coefficients
	 */
	std::vector<float> mDistortion;

	/**
	 * Focal length on x axis in pixels
	 */
	float mFx;

	/**
	 * Focal length on y axis in pixels
	 */
	float mFy;

	/**
	 * Central point coordinates on x axis
	 */
	float mCx;

	/**
	 * Central point coordinates on x axis
	 */
	float mCy;

	/**
	 * Sensor resolution
	 */
	cv::Size mResolution;

	/**
	 * Max floating point coordinate x address value
	 */
	float mMaxX;

	/**
	 * Max floating point coordinate y address value
	 */
	float mMaxY;

	/**
	 * Distortion model used
	 */
	DistortionModel mDistortionModel;

	/**
	 * Distort the Input point according to the Radial Tangential distortion model.
	 *
	 * @tparam Output
	 * @tparam Input
	 * @param point
	 * @return the distorted point in the 3D space
	 */
	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate3D Input>
	[[nodiscard]] Output distortRadialTangential(const Input &point) const {
		const float &k1_ = mDistortion[0];
		const float &k2_ = mDistortion[1];
		const float &p1_ = mDistortion[2];
		const float &p2_ = mDistortion[3];
		// just compute the distorted point
		float x;
		float y;
		if constexpr (concepts::Coordinate3DMembers<Input>) {
			x = point.x / point.z;
			y = point.y / point.z;
		}
		else if constexpr (concepts::Coordinate3DAccessors<Input>) {
			x = point.x() / point.z();
			y = point.y() / point.z();
		}
		const float x_sq     = x * x;
		const float y_sq     = y * y;
		const float xy       = x * y;
		const float sq_sum   = x_sq + y_sq;
		float radialDistance = k1_ * sq_sum + k2_ * sq_sum * sq_sum;
		if (mDistortion.size() == 5) {
			radialDistance += mDistortion[4] * sq_sum * sq_sum * sq_sum;
		}
		const float p_x = x + x * radialDistance + 2.f * p1_ * xy + p2_ * (sq_sum + 2.f * x_sq);
		const float p_y = y + y * radialDistance + 2.f * p2_ * xy + p1_ * (sq_sum + 2.f * y_sq);

		if constexpr (concepts::Coordinate3DMembers<Output>) {
			return Output(p_x * point.z, p_y * point.z, point.z);
		}
		else if constexpr (concepts::Coordinate3DAccessors<Output>) {
			return Output(p_x * point.z(), p_y * point.z(), point.z());
		}
	}

	/**
	 * Distort the Input point according to the Equidistant distortion model.
	 *
	 * @tparam Output
	 * @tparam Input
	 * @param point
	 * @return the distorted point in the 3D space
	 */
	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate3D Input>
	[[nodiscard]] Output distortEquidistant(const Input &point) const {
		float x;
		float y;
		if constexpr (concepts::Coordinate3DMembers<Input>) {
			x = point.x / point.z;
			y = point.y / point.z;
		}
		else if constexpr (concepts::Coordinate3DAccessors<Input>) {
			x = point.x() / point.z();
			y = point.y() / point.z();
		}

		// just compute the distorted point
		const float &k1_ = mDistortion[0];
		const float &k2_ = mDistortion[1];
		const float &k3_ = mDistortion[2];
		const float &k4_ = mDistortion[3];

		const float r2 = x * x + y * y;
		if (r2 < std::numeric_limits<float>::epsilon()) {
			if constexpr (concepts::Coordinate3DMembers<Output>) {
				return Output(point.x, point.y, point.z);
			}
			else if constexpr (concepts::Coordinate3DAccessors<Output>) {
				return Output(point.x(), point.y(), point.z());
			}
		}
		const float r      = std::sqrt(r2);
		const auto theta   = std::atan(r);
		const auto theta2  = theta * theta;
		const auto theta4  = theta2 * theta2;
		const auto theta6  = theta4 * theta2;
		const auto theta8  = theta6 * theta2;
		const auto theta_d = theta * (1 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
		const auto s       = theta_d / r;

		if constexpr (concepts::Coordinate3DMembers<Output>) {
			return Output(s * point.x, s * point.y, point.z);
		}
		else if constexpr (concepts::Coordinate3DAccessors<Output>) {
			return Output(s * point.x(), s * point.y(), point.z());
		}
	}

public:
	using SharedPtr = std::shared_ptr<CameraGeometry>;
	using UniquePtr = std::unique_ptr<CameraGeometry>;

	enum class FunctionImplementation {
		LUT,
		SubPixel
	};

	/**
	 * Create a camera geometry model with distortion model. Currently only radial tangential model
	 * is supported.
	 * @param distortion       Distortion coefficient (4 or 5 coefficient radtan model).
	 * @param fx               Focal length X measured in pixels.
	 * @param fy               Focal length Y measured in pixels.
	 * @param cx               Central point coordinate X in pixels.
	 * @param cy               Central point coordinate Y in pixels.
	 * @param resolution       Sensor resolution.
	 */
	CameraGeometry(const std::vector<float> &distortion, const float fx, const float fy, const float cx, const float cy,
		const cv::Size &resolution, const DistortionModel distortionModel) :
		mDistortion(distortion),
		mFx(fx),
		mFy(fy),
		mCx(cx),
		mCy(cy),
		mResolution(resolution),
		mMaxX(static_cast<float>(mResolution.width - 1)),
		mMaxY(static_cast<float>(mResolution.height - 1)),
		mDistortionModel((distortionModel)) {
		if ((mDistortionModel == DistortionModel::RadTan) && ((mDistortion.size() < 4) || (mDistortion.size() > 5))) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"radialTangential\" model must be 4 or 5.", mDistortion.size());
		}
		else if ((mDistortionModel == DistortionModel::Equidistant) && (mDistortion.size() != 4)) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"equidistant\" model must be 4.", mDistortion.size());
		}
		else if ((mDistortionModel == DistortionModel::None) && (!mDistortion.empty())) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"none\" model must be 0.", mDistortion.size());
		}

		generateLUTs();
	}

	/**
	 * Create a camera geometry model without distortion model. Currently only radial tangential model
	 * is supported.
	 *
	 * Any calls to function dependent on distortion will cause exceptions or segfaults.
	 * @param fx               Focal length X measured in pixels.
	 * @param fy               Focal length Y measured in pixels.
	 * @param cx               Central point coordinate X in pixels.
	 * @param cy               Central point coordinate Y in pixels.
	 * @param resolution       Sensor resolution.
	 */
	CameraGeometry(const float fx, const float fy, const float cx, const float cy, const cv::Size &resolution) :
		mFx(fx),
		mFy(fy),
		mCx(cx),
		mCy(cy),
		mResolution(resolution),
		mMaxX(static_cast<float>(mResolution.width - 1)),
		mMaxY(static_cast<float>(mResolution.height - 1)),
		mDistortionModel(DistortionModel::None) {
		generateLUTs();
	}

	/**
	 * Returns pixel coordinates of given point with applied back projection, undistortion, and projection.
	 * This function uses look-up table and is designed for minimal execution speed.
	 *
	 * WARNING: will cause a segfault if coordinates are out-of-bounds or if distortion model
	 *          is not available.
	 * @param point     Pixel coordinate
	 * @return          Undistorted pixel coordinate
	 */
	template<concepts::Coordinate2DCostructible Output, concepts::Coordinate2D Input>
	[[nodiscard]] inline Output undistort(const Input &point) const {
		dv::runtime_assert(
			isUndistortionAvailable(), "Call to undistort method of a camera geometry without distortion model");
		dv::runtime_assert(isWithinDimensions(point), "Undistortion coordinates are out of bounds");
		size_t address;
		if constexpr (concepts::Coordinate2DMembers<Input>) {
			address = static_cast<size_t>(
				(static_cast<int32_t>(point.y) * mResolution.width) + static_cast<int32_t>(point.x));
		}
		else if constexpr (concepts::Coordinate2DAccessors<Input>) {
			address = static_cast<size_t>(
				(static_cast<int32_t>(point.y()) * mResolution.width) + static_cast<int32_t>(point.x()));
		}
		const auto p = mDistortionPixelLUT[address];
		return Output(p.x, p.y);
	}

	/**
	 * Undistort event coordinates, discards events which fall beyond camera resolution.
	 * @param events        Input events
	 * @return              A new event store containing the same events with undistorted coordinates
	 */
	[[nodiscard]] dv::EventStore undistortEvents(const dv::EventStore &events) const {
		if (mDistortionLUT.empty()) {
			throw std::domain_error(
				"Trying to undistort events with a camera geometry without distortion coefficients");
		}

		dv::EventStore output;
		for (const auto &event : events) {
			if (const auto point = undistort<cv::Point2i>(event); isWithinDimensions(point)) {
				output.emplace_back(
					event.timestamp(), static_cast<int16_t>(point.x), static_cast<int16_t>(point.y), event.polarity());
			}
		}

		return output;
	}

	/**
	 * Undistort point coordinates.
	 * @param coordinates   Input point coordinates
	 * @return              A new vector containing the points with undistorted coordinates
	 */
	template<concepts::Coordinate2DMutableIterable Output, concepts::Coordinate2DIterable Input>
	[[nodiscard]] Output undistortSequence(const Input &coordinates) const {
		if (mDistortionLUT.empty()) {
			throw std::domain_error(
				"Trying to undistort points with a camera geometry without distortion coefficients");
		}
		Output undistorted;
		undistorted.reserve(coordinates.size());
		for (const auto &coordinate : coordinates) {
			undistorted.push_back(undistort<concepts::iterable_element_type<Output>>(coordinate));
		}
		return undistorted;
	}

	/**
	 * Apply distortion to a 3D point.
	 * @param point         Point in 3D space
	 * @return              Distorted point
	 */
	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate3D Input>
	[[nodiscard]] Output distort(const Input &undistortedPoint) const {
		dv::runtime_assert(isUndistortionAvailable(),
			"Trying to apply distortion with a camera geometry without distortion coefficients");
		switch (mDistortionModel) {
			case DistortionModel::RadTan:
				return distortRadialTangential<Output, Input>(undistortedPoint);
			case DistortionModel::Equidistant:
				return distortEquidistant<Output, Input>(undistortedPoint);
			case DistortionModel::None: {
				if constexpr (dv::concepts::Coordinate3DAccessors<Input>) {
					return Output(undistortedPoint.x(), undistortedPoint.y(), undistortedPoint.z());
				}
				else {
					return Output(undistortedPoint.x, undistortedPoint.y, undistortedPoint.z);
				}
			}
			default:
				throw dv::exceptions::InvalidArgument<int>(
					"Invalid distortion model", dv::EnumAsInteger(mDistortionModel));
		}
	}

	/**
	 * Apply direct distortion on the 3D points.
	 * @param points        Input points
	 * @return              Distorted points
	 */
	template<concepts::Coordinate3DMutableIterable Output, concepts::Coordinate3DIterable Input>
	[[nodiscard]] Output distortSequence(const Input &points) const {
		if (!isUndistortionAvailable()) {
			throw std::domain_error(
				"Trying to apply distortion with a camera geometry without distortion coefficients");
		}
		Output output;
		for (const auto &point : points) {
			output.push_back(distort<concepts::iterable_element_type<Output>>(point));
		}
		return output;
	}

	/**
	 * Back-project pixel coordinates into a unit ray vector of depth = 1.0 meters.
	 * @param pixel         	Pixel to be projected
	 * @return					Back projected unit ray
	 * @tparam implementation	Specify the internal implementation to performthe computations, `SubPixel` performs all
	 * 							computations without any optimization, `LUT` option	avoids computation by perfoming a
	 * 							look-up table operation instead, but rounds input coordinate values.
	 */
	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate2D Input,
		FunctionImplementation implementation = FunctionImplementation::LUT>
	[[nodiscard]] Output backProject(const Input &pixel) const {
		if constexpr (implementation == FunctionImplementation::LUT) {
			if constexpr (concepts::Coordinate2DMembers<Input>) {
				dv::runtime_assert(isWithinDimensions(pixel), "Back projection with out-of-bounds pixel coordinates.");
				const cv::Point3f &p = mBackProjectLUT[static_cast<size_t>(
					(static_cast<int>(pixel.y) * mResolution.width) + static_cast<int>(pixel.x))];
				return Output(p.x, p.y, p.z);
			}
			else if constexpr (concepts::Coordinate2DAccessors<Input>) {
				dv::runtime_assert(isWithinDimensions(pixel), "Back projection with out-of-bounds pixel coordinates.");
				const cv::Point3f &p = mBackProjectLUT[static_cast<size_t>(
					(static_cast<int>(pixel.y()) * mResolution.width) + static_cast<int>(pixel.x()))];
				return Output(p.x, p.y, p.z);
			}
		}
		else {
			if constexpr (concepts::Coordinate2DMembers<Input>) {
				return Output((pixel.x - mCx) / mFx, (pixel.y - mCy) / mFy, 1);
			}
			else if (concepts::Coordinate2DAccessors<Input>) {
				return Output((pixel.x() - mCx) / mFx, (pixel.y() - mCy) / mFy, 1);
			}
		}
	}

	/**
	 * Back project a sequence of 2D point into 3D unit ray-vectors.
	 * @param points        	Input points.
	 * @return              	A sequence of back-projected unit ray vectors.
	 * @tparam implementation	Specify the internal implementation to performthe computations, `SubPixel` performs all
	 * 							computations without any optimization, `LUT` option	avoids computation by perfoming a
	 * 							look-up table operation instead, but rounds input coordinate values.
	 */
	template<concepts::Coordinate3DMutableIterable Output, concepts::Coordinate2DIterable Input,
		FunctionImplementation implementation = FunctionImplementation::LUT>
	[[nodiscard]] Output backProjectSequence(const Input &points) const {
		Output output;
		output.reserve(points.size());
		for (const auto &point : points) {
			output.push_back(backProject<concepts::iterable_element_type<Output>,
				concepts::iterable_element_type<Input>, implementation>(point));
		}
		return output;
	}

	/**
	 * Returns a unit ray of given coordinates with applied back projection and undistortion.
	 * This function uses look-up table and is designed for minimal execution speed.
	 *
	 * WARNING: will cause a segfault if coordinates are out-of-bounds or if distortion model
	 *          is not available.
	 * @param pixel     Pixel coordinate
	 * @return          Back projected and undistorted unit ray
	 */
	template<concepts::Coordinate3DCostructible Output, concepts::Coordinate2D Input>
	[[nodiscard]] inline Output backProjectUndistort(const Input &pixel) const {
		dv::runtime_assert(
			isUndistortionAvailable(), "Call to undistort method of a camera geometry without distortion model");
		dv::runtime_assert(isWithinDimensions(pixel), "Undistortion coordinates are out of bounds");
		size_t address;
		if constexpr (concepts::Coordinate2DMembers<Input>) {
			address
				= static_cast<size_t>(pixel.y) * static_cast<size_t>(mResolution.width) + static_cast<size_t>(pixel.x);
		}
		else if constexpr (concepts::Coordinate2DAccessors<Input>) {
			address = static_cast<size_t>(pixel.y()) * static_cast<size_t>(mResolution.width)
					+ static_cast<size_t>(pixel.x());
		}
		const auto p = mDistortionLUT[address];
		return Output(p.x, p.y, p.z);
	}

	/**
	 * Undistort and back project a batch of points. Output is normalized point coordinates as unit rays.
	 * @param points        Input points.
	 * @return              Undistorted and back projected points.
	 */
	template<concepts::Coordinate3DMutableIterable Output, concepts::Coordinate2DIterable Input>
	[[nodiscard]] Output backProjectUndistortSequence(const Input &points) const {
		if (!isUndistortionAvailable()) {
			throw std::domain_error(
				"Trying to apply distortion with a camera geometry without distortion coefficients");
		}

		Output undistorted;
		for (const auto &p : points) {
			undistorted.push_back(backProjectUndistort<concepts::iterable_element_type<Output>>(p));
		}
		return undistorted;
	}

	/**
	 * Project a 3D point into pixel plane.
	 *
	 * WARNING: Does not perform range checking!
	 * @param points        3D points to be projected
	 * @return              Projected pixel coordinates
	 */
	template<concepts::Coordinate2DCostructible Output, concepts::Coordinate3D Input>
	[[nodiscard]] Output project(const Input &points) const {
		if constexpr (concepts::Coordinate3DMembers<Input>) {
			return Output(((points.x / points.z) * mFx) + mCx, ((points.y / points.z) * mFy) + mCy);
		}
		else if constexpr (concepts::Coordinate3DAccessors<Input>) {
			return Output(((points.x() / points.z()) * mFx) + mCx, ((points.y() / points.z()) * mFy) + mCy);
		}
	}

	/**
	 * Project a batch of 3D points into pixel plane.
	 * @param points            Points to be projected.
	 * @param dimensionCheck    Whether to perform resolution check, if true, output points outside of valid
	 *                          frame resolution will be omitted. If disabled, output point count and order will be
	 *                          the same as input points.
	 * @return                  Projected points in pixel plane.
	 */
	template<concepts::Coordinate2DMutableIterable Output, concepts::Coordinate3DIterable Input>
	[[nodiscard]] Output projectSequence(const Input &points, const bool dimensionCheck = true) const {
		Output projected;
		projected.reserve(points.size());

		if (dimensionCheck) {
			for (const auto &p : points) {
				if (const auto projection = project<concepts::iterable_element_type<Output>>(p);
					isWithinDimensions(projection)) {
					projected.push_back(std::move(projection));
				}
			}
			projected.shrink_to_fit();
		}
		else {
			for (const auto &p : points) {
				projected.push_back(project<concepts::iterable_element_type<Output>>(p));
			}
		}

		return projected;
	}

	/**
	 * Check whether given coordinates are within valid range.
	 * @param point		Pixel coordinates
	 * @return          True if the coordinate values are within camera resolution,
	 *                  false otherwise.
	 */
	template<concepts::Coordinate2D Input>
	[[nodiscard]] inline bool isWithinDimensions(const Input &point) const {
		if constexpr (concepts::Coordinate2DMembers<Input>) {
			if constexpr (std::floating_point<decltype(point.x)>) {
				return point.x >= 0 && point.x <= mMaxX && point.y >= 0 && point.y <= mMaxY;
			}
			else {
				return point.x >= 0 && point.x < mResolution.width && point.y >= 0 && point.y < mResolution.height;
			}
		}
		else if constexpr (concepts::Coordinate2DAccessors<Input>) {
			if constexpr (std::floating_point<decltype(point.x())>) {
				return point.x() >= 0 && point.x() <= mMaxX && point.y() >= 0 && point.y() <= mMaxY;
			}
			else {
				return point.x() >= 0 && point.x() < mResolution.width && point.y() >= 0
					&& point.y() < mResolution.height;
			}
		}
	}

	/**
	 * Checks whether this camera geometry calibration contains coefficient for an undistortion model.
	 * @return	True if undistortion is available, false otherwise
	 */
	[[nodiscard]] inline bool isUndistortionAvailable() const {
		return !mDistortionLUT.empty();
	}

	/**
	 * Get camera matrix in the format:
	 * | mFx  0 mCx |
	 * |  0 mFy mCy |
	 * |  0  0  1 |
	 * @return      3x3 Camera matrix with pixel length values
	 */
	[[nodiscard]] cv::Matx33f getCameraMatrix() const {
		cv::Matx33f camMat;
		camMat << mFx, 0.f, mCx, 0.f, mFy, mCy, 0.f, 0.f, 1.f;
		return camMat;
	}

	/**
	 * Focal length
	 * @return      Focal length in pixels
	 */
	template<concepts::Coordinate2DCostructible Output = cv::Point2f>
	[[nodiscard]] Output getFocalLength() const {
		return Output(mFx, mFy);
	}

	/**
	 * Central point coordinates
	 * @return      Central point coordinates in pixels
	 */
	template<concepts::Coordinate2DCostructible Output = cv::Point2f>
	[[nodiscard]] Output getCentralPoint() const {
		return Output(mCx, mCy);
	}

	/**
	 * Get distortion coefficients
	 * @return      Vector containing distortion coefficients
	 */
	[[nodiscard]] std::vector<float> getDistortion() const {
		return mDistortion;
	}

	/**
	 * Get distortion model
	 * @return DistortionModel type
	 */
	[[nodiscard]] DistortionModel getDistortionModel() const {
		return mDistortionModel;
	}

	/**
	 * Get the camera resolution.
	 * @return 	Camera sensor resolution
	 */
	[[nodiscard]] cv::Size getResolution() const {
		return mResolution;
	}
};

} // namespace dv::camera

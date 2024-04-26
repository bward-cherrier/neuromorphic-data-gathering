#pragma once

#include "../camera/stereo_geometry.hpp"
#include "../core/concepts.hpp"
#include "../core/frame.hpp"
#include "utils.hpp"

namespace dv {

/**
 * Semi-dense stereo matcher - a class that performs disparity calculation using an OpenCV dense disparity
 * calculation algorithm. The implementation performs accumulation of a stereo pair of images of input events
 * and applies the given stereo disparity matcher algorithm (semi-global block matching by default).
 */
template<dv::concepts::EventToFrameConverter<dv::EventStore> AccumulatorClass = dv::EdgeMapAccumulator>
class SemiDenseStereoMatcher {
protected:
	std::shared_ptr<cv::StereoMatcher> mMatcher         = nullptr;
	std::unique_ptr<AccumulatorClass> mLeftAccumulator  = nullptr;
	std::unique_ptr<AccumulatorClass> mRightAccumulator = nullptr;
	dv::Frame mLeftFrame;
	dv::Frame mRightFrame;

	std::unique_ptr<dv::camera::StereoGeometry> mStereoGeometry = nullptr;

private:
	/**
	 * Validates stereo geometry pointer, throws an error if the value is unset.
	 */
	void validateStereoGeometry() const {
		if (mStereoGeometry == nullptr) {
			throw dv::exceptions::RuntimeError("Depth estimation requires known stereo geometry which is "
											   "retrieved by calibrating the stereo camera. Construct the matcher by "
											   "providing a `dv::camera::StereoGeometry` object to enable depth "
											   "estimation.");
		}
	}

public:
	/**
	 * Construct a semi dense stereo matcher object by providing custom accumulators for left and right cameras
	 * and a stereo matcher class.
	 * @param leftAccumulator 	Accumulator for the left camera.
	 * @param rightAccumulator 	Accumulator for the right camera.
	 * @param matcher 			Stereo matcher algorithm, if not provided, the implementation will use a
	 * 							cv::StereoSGBM class with default parameters.
	 */
	SemiDenseStereoMatcher(std::unique_ptr<AccumulatorClass> leftAccumulator,
		std::unique_ptr<AccumulatorClass> rightAccumulator,
		const std::shared_ptr<cv::StereoMatcher> &matcher = cv::StereoSGBM::create()) :
		mMatcher(matcher),
		mLeftAccumulator(std::move(leftAccumulator)),
		mRightAccumulator(std::move(rightAccumulator)) {
	}

	/**
	 * Construct a semi dense stereo matcher with default accumulator settings and a stereo matcher class.
	 * @param leftResolution 	Resolution of the left camera.
	 * @param rightResolution 	Resolution of the right camera.
	 * @param matcher 			Stereo matcher algorithm, if not provided, the implementation will use a
	 * 							cv::StereoSGBM class with default parameters.
	 */
	explicit SemiDenseStereoMatcher(const cv::Size &leftResolution, const cv::Size &rightResolution,
		const std::shared_ptr<cv::StereoMatcher> &matcher = cv::StereoSGBM::create()) :
		SemiDenseStereoMatcher(std::make_unique<AccumulatorClass>(leftResolution),
			std::make_unique<AccumulatorClass>(rightResolution), matcher) {
	}

	/**
	 * Construct a semi dense stereo matcher with default accumulator settings and a stereo matcher class. The
	 * calibration information about camera will be extracted from the stereo geometry class instance.
	 * @param geometry 	Object describing the stereo camera geometry.
	 * @param matcher 	Stereo matcher algorithm, if not provided, the implementation will use a
	 * 					cv::StereoSGBM class with optimized parameters.
	 */
	explicit SemiDenseStereoMatcher(std::unique_ptr<dv::camera::StereoGeometry> geometry,
		std::shared_ptr<cv::StereoMatcher> matcher = dv::depth::defaultStereoMatcher()) :
		SemiDenseStereoMatcher(std::make_unique<AccumulatorClass>(geometry->getLeftCameraGeometry().getResolution()),
			std::make_unique<AccumulatorClass>(geometry->getRightCameraGeometry().getResolution()),
			std::move(matcher)) {
		mStereoGeometry = std::move(geometry);
	}

	/**
	 * Construct a semi dense stereo matcher object by providing custom accumulators for left and right cameras
	 * and a stereo matcher class. The calibration information about camera will be extracted from the stereo
	 * geometry class instance.
	 * @param geometry			Object describing the stereo camera geometry.
	 * @param leftAccumulator 	Accumulator for the left camera.
	 * @param rightAccumulator 	Accumulator for the right camera.
	 * @param matcher 			Stereo matcher algorithm, if not provided, the implementation will use a
	 * 							cv::StereoSGBM class with optimized parameters.
	 */
	SemiDenseStereoMatcher(std::unique_ptr<dv::camera::StereoGeometry> geometry,
		std::unique_ptr<AccumulatorClass> leftAccumulator, std::unique_ptr<AccumulatorClass> rightAccumulator,
		std::shared_ptr<cv::StereoMatcher> matcher = dv::depth::defaultStereoMatcher()) :
		mMatcher(std::move(matcher)),
		mLeftAccumulator(std::move(leftAccumulator)),
		mRightAccumulator(std::move(rightAccumulator)),
		mStereoGeometry(std::move(geometry)) {
	}

	/**
	 * Compute disparity of the two given event stores. The events will be accumulated using the
	 * accumulators for left and right camera accordingly and disparity is computed using the configured
	 * block matching algorithm. The function is not going to slice the input events, so event streams
	 * have to be synchronized and sliced accordingly. The `dv::StereoEventStreamSlicer` class is a good
	 * option for slicing stereo event streams.
	 *
	 * NOTE: Accumulated frames will be rectified only if a stereo geometry class was provided during
	 * construction.
	 * @param left 		Events from left camera.
	 * @param right 	Events from right camera.
	 * @return 			Disparity map computed by the configured block matcher.
	 * @sa dv::StereoEventStreamSlicer for synchronized slicing of a stereo event stream.
	 */
	[[nodiscard]] cv::Mat computeDisparity(const dv::EventStore &left, const dv::EventStore &right) {
		dv::runtime_assert(mLeftAccumulator != nullptr, "Left camera accumulator is unset!");
		dv::runtime_assert(mRightAccumulator != nullptr, "Right camera accumulator is unset!");
		dv::runtime_assert(mMatcher != nullptr, "Stereo mMatcher is unset!");

		mLeftAccumulator->accept(left);
		mLeftFrame = mLeftAccumulator->generateFrame();

		mRightAccumulator->accept(right);
		mRightFrame = mRightAccumulator->generateFrame();

		return compute(mLeftFrame.image, mRightFrame.image);
	}

	/**
	 * Compute stereo disparity given a time synchronized pair of images. Images will be rectified before computing
	 * disparity if a StereoGeometry class instance was provided.
	 * @param leftImage		Left image of a stereo pair of images.
	 * @param rightImage 	Right image of a stereo pair of images.
	 * @return 				Disparity map computed by the configured block matcher.
	 */
	[[nodiscard]] cv::Mat compute(const cv::Mat &leftImage, const cv::Mat &rightImage) const {
		cv::Mat disparity;

		if (mStereoGeometry == nullptr) {
			mMatcher->compute(leftImage, rightImage, disparity);
		}
		else {
			mMatcher->compute(mStereoGeometry->remapImage(camera::StereoGeometry::CameraPosition::Left, leftImage),
				mStereoGeometry->remapImage(camera::StereoGeometry::CameraPosition::Right, rightImage), disparity);
		}

		return disparity;
	}

	/**
	 * Retrieve the accumulated frame from the left camera event stream.
	 * @return 	An accumulated frame.
	 */
	[[nodiscard]] const dv::Frame &getLeftFrame() const {
		return mLeftFrame;
	}

	/**
	 * Retrieve the accumulated frame from the right camera event stream.
	 * @return 	An accumulated frame.
	 */
	[[nodiscard]] const dv::Frame &getRightFrame() const {
		return mRightFrame;
	}

	/**
	 * Estimate depth given the disparity map and a list of events. The coordinates will be rectified and
	 * a disparity value will be looked up in the disparity map. The depth of each event is calculated
	 * using an equation: depth = (focalLength * baseline) / (disparity * pixelPitch). Focal length is expressed
	 * in meter distance.
	 *
	 * The function requires knowledge about the pixel pitch distance which needs to be provided prior to
	 * calculations. The pixel pitch can be available in the camera calibration (in this case it will be looked up
	 * during construction of the class). If the pixel pitch is not available there, it must be provided manually
	 * using the `setPixelPitch` method. The pixel pitch value can be looked up in `dv::io::CameraCapture` class
	 * in case if running the stereo estimation in a live camera scenario.
	 *
	 * For practical applications, depth estimation should be evaluated prior to any use. The directly estimated depth
	 * values can contain measurable errors which should be accounted for - the errors can usually be within 10-20%
	 * fixed absolute error distance. Usually this comes from various inaccuracies and can be mitigated by introducing
	 * a correction factor for the depth estimate.
	 * @param disparity		Disparity map.
	 * @param events			Input events.
	 * @param disparityScale	Scale of disparity value in the disparity map, if subpixel accuracy is enabled
	 * 						in the block matching, this value will be equal to 16.
	 * @return					A depth event store, the events will contain the same information as in the
	 * 						input, but additionally will have the depth value. Events whose coordinates
	 * 						are outside of image bounds after rectification will be skipped.
	 */
	[[nodiscard]] inline dv::DepthEventStore estimateDepth(
		const cv::Mat &disparity, const dv::EventStore &events, const float disparityScale = 16.f) const {
		validateStereoGeometry();
		return mStereoGeometry->estimateDepth(disparity, events, disparityScale);
	}

	/**
	 * Convert a disparity map into a depth frame. Each disparity value is converted into depth using the equation
	 * depth = (focalLength * baseline) / (disparity * pixelPitch). Output frame contains distance values expressed
	 * in integer values of millimeter distance.
	 * @param disparity 		Input disparity map.
	 * @param disparityScale	Scale of disparity value in the disparity map, if subpixel accuracy is enabled
	 * 						in the block matching, this value will be equal to 16.
	 * @return 					A converted depth frame.
	 */
	[[nodiscard]] inline dv::DepthFrame estimateDepthFrame(
		const cv::Mat &disparity, const float disparityScale = 16.f) const {
		validateStereoGeometry();
		return mStereoGeometry->toDepthFrame(disparity, disparityScale);
	}
};

} // namespace dv

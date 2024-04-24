#pragma once

#include "../data/utilities.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "../kinematics/motion_compensator.hpp"
#include "image_pyramid.hpp"
#include "redetection_strategy.hpp"
#include "tracker_base.hpp"

#include <utility>

namespace dv::features {

/**
 * Lucas-Kanade tracker configuration parameters.
 */
struct LucasKanadeConfig {
	/**
	 * Generate a mask which would disable image regions where features
	 * are already succesfully tracked.
	 */
	bool maskedFeatureDetect = true;

	/**
	 * Tracking termination criteria for the LK tracker.
	 */
	double terminationEpsilon = 0.1;

	/**
	 * Total number of pyramid layers used by the LK tracker.
	 */
	int numPyrLayers = 2;

	/**
	 * Size of the search around the tracked feature.
	 */
	cv::Size searchWindowSize = cv::Size(24, 24);
};

/**
 * A feature based sparse Lucas-Kanade feature tracker based on image pyramids.
 */
class ImageFeatureLKTracker : public TrackerBase {
public:
	using Config    = LucasKanadeConfig;
	using SharedPtr = std::shared_ptr<ImageFeatureLKTracker>;
	using UniquePtr = std::unique_ptr<ImageFeatureLKTracker>;

protected:
	Config mConfig = {};

	RedetectionStrategy::UniquePtr mRedetectionStrategy = nullptr;

	ImagePyrFeatureDetector::UniquePtr mDetector = nullptr;

	cv::Ptr<cv::SparsePyrLKOpticalFlow> mTracker;

	ImagePyramid::UniquePtr mPreviousFrame = nullptr;

	ImagePyramid::UniquePtr mCurrentFrame = nullptr;

	kinematics::PixelMotionPredictor::UniquePtr mPredictor = nullptr;

	std::unique_ptr<kinematics::LinearTransformerf> mTransformer = nullptr;

	std::map<int64_t, float> mDepthHistory;

	camera::CameraGeometry::SharedPtr mCamera = nullptr;

	cv::Size mResolution;

	bool mLookbackRejection = false;

	float mRejectionDistanceThreshold = 10.f;

	// 5 Seconds of depth history
	const int64_t depthHistoryDuration = 5000000;

	float constantDepth = 3.f;

	[[nodiscard]] std::vector<cv::Point2f> predictNextPoints(
		const int64_t previousTime, const std::vector<cv::Point2f> &previousPoints, const int64_t nextTime) {
		// The class does not contain predictor or no transform can be a case when the external
		// estimator is not initialized, so the MotionAware tracker will behave as a regular tracker. Return
		// a copy of the previous point locations.
		if (!mPredictor || !mTransformer) {
			return previousPoints;
		}

		// Extract camera poses estimated externally
		auto T_WC0 = mTransformer->getTransformAt(previousTime);
		auto T_WC1 = mTransformer->getTransformAt(nextTime);

		// Transforms are not yet available
		if (!T_WC0 || !T_WC1) {
			return previousPoints;
		}

		// 3 meter depth is a quite a reasonable guess for the estimator, a rough estimate also provide decent overall
		// result in practice.
		float depth = constantDepth;
		if (!mDepthHistory.empty()) {
			// Find closest depth, even if it was estimated some time ago
			auto closestDepth = mDepthHistory.upper_bound(nextTime);
			depth = closestDepth != mDepthHistory.end() ? closestDepth->second : mDepthHistory.rbegin()->second;
		}

		// Delta transform from T_WC0 (prev) -> T_WC1 (next)
		auto T_C0_C1 = T_WC0->delta(*T_WC1);
		// Predict poses of the previous points
		std::vector<cv::Point2f> prediction;
		prediction.reserve(previousPoints.size());

		for (const auto &p : previousPoints) {
			const auto predicted = mPredictor->predict<cv::Point2f>(p, T_C0_C1, depth);
			if (mCamera->isWithinDimensions(predicted)) {
				prediction.push_back(predicted);
			}
			else {
				prediction.push_back(p);
			}
		}
		return prediction;
	}

	/**
	 * Perform the LK tracking.
	 * @return      Result of the tracking.
	 */
	[[nodiscard]] Result::SharedPtr track() override {
		dv::runtime_assert(static_cast<bool>(mCurrentFrame), "Current frame is not passed!");

		// Initialize if we do not have any features
		if (!mPreviousFrame || lastFrameResults->keypoints.empty()) {
			TrackerBase::Result::SharedPtr result = std::make_shared<TrackerBase::Result>(
				mCurrentFrame->timestamp, mDetector->runDetection(*mCurrentFrame, maxTracks), true);
			mPreviousFrame = std::move(mCurrentFrame);
			return result;
		}

		// Perform LK tracking
		std::vector<unsigned char> status;
		std::vector<float> err;

		std::vector<cv::Point2f> previousKpts = dv::data::convertToCvPoints(lastFrameResults->keypoints);
		std::vector<cv::Point2f> predictedKpts
			= predictNextPoints(lastFrameResults->timestamp, previousKpts, mCurrentFrame->timestamp);

		// This should never be the case, but let's be careful
		if (predictedKpts.empty()) {
			predictedKpts = previousKpts;
		}

		mTracker->calc(mPreviousFrame->pyramid, mCurrentFrame->pyramid, previousKpts, predictedKpts, status, err);

		// Perform backward tracking and validate that tracking corresponds
		if (mLookbackRejection) {
			std::vector<unsigned char> lbStatus;
			std::vector<float> lbErr;
			// Performing this without actual prediction value
			std::vector<cv::Point2f> lbKpts = predictedKpts;

			mTracker->calc(mCurrentFrame->pyramid, mPreviousFrame->pyramid, predictedKpts, lbKpts, lbStatus, lbErr);
			for (size_t i = 0; i < lbKpts.size(); i++) {
				// Select good points
				const auto &kpt = lbKpts[i];
				// If look-back tracking failed or tracked distance does not correspond with 5 pixel radius -- reject
				if (lbStatus[i] != 1 || cv::norm((kpt - predictedKpts[i])) > mRejectionDistanceThreshold) {
					status[i] = 0;
				}
			}
		}

		// Fill in the keypoints
		dv::cvector<dv::TimedKeyPoint> keyPoints;
		for (size_t i = 0; i < predictedKpts.size(); i++) {
			// Select good points
			const auto &kpt = predictedKpts[i];

			if (status[i] == 1 && mDetector->isWithinROI(kpt)) {
				const auto &feat = lastFrameResults->keypoints[i];
				keyPoints.emplace_back(dv::Point2f(kpt.x, kpt.y), feat.size, feat.angle, err[i], feat.octave,
					feat.class_id, mCurrentFrame->timestamp);
			}
		}

		bool asKeyframe = false;

		// Decide on redetection of features
		if (mRedetectionStrategy->decideRedetection(*this)) {
			cv::Mat mask;
			if (mConfig.maskedFeatureDetect) {
				// Disable search on locations where we already have features
				mask = cv::Mat(mCurrentFrame->pyramid.at(0).size(), CV_8UC1, cv::Scalar(255));
				for (const auto &feat : keyPoints) {
					cv::circle(mask, cv::Point2f(feat.pt.x(), feat.pt.y()),
						static_cast<int>(std::ceil(feat.size / 2.f)), cv::Scalar(0), -1);
				}
			}
			// Detect new features
			std::vector<cv::Point2f> pts;
			mDetector->runRedetection(keyPoints, *mCurrentFrame, maxTracks, mask);
			asKeyframe = true;
		}

		mPreviousFrame = std::move(mCurrentFrame);
		return std::make_shared<TrackerBase::Result>(mPreviousFrame->timestamp, keyPoints, asKeyframe);
	}

	/**
	 * Construct a tracker with default detector parameters, but configurable tracker parameters.
	 * @param resolution        Image resolution.
	 * @param _config           Lucas-Kanade tracker parameters.
	 */
	ImageFeatureLKTracker(const cv::Size &resolution, const Config &config) : mConfig(config), mResolution(resolution) {
		mTracker = cv::SparsePyrLKOpticalFlow::create(mConfig.searchWindowSize, mConfig.numPyrLayers - 1,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, mConfig.terminationEpsilon),
			cv::OPTFLOW_USE_INITIAL_FLOW);
		setDetector(nullptr);
		setRedetectionStrategy(nullptr);
	}

	/**
	 * Construct a tracker with default detector parameters, but configurable tracker parameters.
	 * @param resolution        Image resolution.
	 * @param _config           Lucas-Kanade tracker parameters.
	 */
	ImageFeatureLKTracker(const camera::CameraGeometry::SharedPtr &cameraGeometry, const Config &config) :
		mConfig(config),
		mCamera(cameraGeometry),
		mResolution(cameraGeometry->getResolution()) {
		mTracker = cv::SparsePyrLKOpticalFlow::create(mConfig.searchWindowSize, mConfig.numPyrLayers - 1,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, mConfig.terminationEpsilon),
			cv::OPTFLOW_USE_INITIAL_FLOW);
		setDetector(nullptr);
		setRedetectionStrategy(nullptr);
		setMotionPredictor(nullptr);
		mTransformer = std::make_unique<kinematics::LinearTransformerf>(10000);
	}

public:
	[[nodiscard]] static ImageFeatureLKTracker::UniquePtr RegularTracker(const cv::Size &resolution,
		const Config &_config = Config(), ImagePyrFeatureDetector::UniquePtr detector = nullptr,
		RedetectionStrategy::UniquePtr redetection = nullptr) {
		ImageFeatureLKTracker::UniquePtr tracker{new ImageFeatureLKTracker(resolution, _config)};
		if (detector) {
			tracker->setDetector(std::move(detector));
		}
		if (redetection) {
			tracker->setRedetectionStrategy(std::move(redetection));
		}
		return tracker;
	}

	[[nodiscard]] static ImageFeatureLKTracker::UniquePtr MotionAwareTracker(
		const camera::CameraGeometry::SharedPtr &camera, const Config &config = Config(),
		kinematics::PixelMotionPredictor::UniquePtr motionPredictor = nullptr,
		ImagePyrFeatureDetector::UniquePtr detector = nullptr, RedetectionStrategy::UniquePtr redetection = nullptr) {
		ImageFeatureLKTracker::UniquePtr tracker{new ImageFeatureLKTracker(camera, config)};
		if (detector) {
			tracker->setDetector(std::move(detector));
		}
		if (redetection) {
			tracker->setRedetectionStrategy(std::move(redetection));
		}
		if (motionPredictor) {
			tracker->setMotionPredictor(std::move(motionPredictor));
		}
		return tracker;
	}

	/**
	 * Add an input image for the tracker. Image pyramid will be built from the given image.
	 * @param image             Acquired image.
	 */
	virtual void accept(const dv::Frame &image) {
		if (mPreviousFrame && image.timestamp < mPreviousFrame->timestamp) {
			throw dv::exceptions::InvalidArgument<cv::Mat>("Image from the past was pushed into tracker input");
		}
		if ((image.image.cols != mResolution.width) || (image.image.rows != mResolution.height)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format(
					"Resolution mismatch. Check camera config and input data. Expected image dimensions: [{}x{}]",
					mResolution.width, mResolution.height),
				fmt::format("{}x{}", image.image.cols, image.image.rows));
		}

		if (image.image.channels() > 1) {
			cv::Mat grayscale;
			cv::cvtColor(image.image, grayscale, cv::COLOR_BGR2GRAY);
			mCurrentFrame = std::make_unique<ImagePyramid>(
				image.timestamp, grayscale, mConfig.searchWindowSize, mConfig.numPyrLayers - 1);
		}
		else {
			mCurrentFrame = std::make_unique<ImagePyramid>(
				image.timestamp, image.image, mConfig.searchWindowSize, mConfig.numPyrLayers - 1);
		}
	}

	/**
	 * Set a new redetection strategy.
	 * @param redetectionStrategy 		Redetection strategy instance.
	 * @deprecated Use setRedetectionStrategy instead
	 */
	[[deprecated("Use setRedetectionStrategy instead")]] void setRedectionStrategy(
		RedetectionStrategy::UniquePtr redetectionStrategy) {
		setRedetectionStrategy(std::move(redetectionStrategy));
	}

	/**
	 * Set a new redetection strategy.
	 * @param redetectionStrategy 		Redetection strategy instance.
	 */
	void setRedetectionStrategy(RedetectionStrategy::UniquePtr redetectionStrategy) {
		if (redetectionStrategy) {
			mRedetectionStrategy = std::move(redetectionStrategy);
		}
		else {
			mRedetectionStrategy = std::make_unique<FeatureCountRedetection>(0.5f);
		}
	}

	/**
	 * Set a new feature (corner) detector. If a `nullptr` is passed, the
	 * function will instantiate a feature detector with no parameters (defaults).
	 * @param detector 				Feature detector instance.
	 */
	void setDetector(ImagePyrFeatureDetector::UniquePtr detector) {
		if (detector) {
			mDetector = std::move(detector);
		}
		else {
			mDetector = std::make_unique<dv::features::FeatureDetector<dv::features::ImagePyramid, cv::Feature2D>>(
				mResolution, cv::GFTTDetector::create());
		}
	}

	/**
	 * Set new pixel motion predictor instance. If a `nullptr` is passed, the
	 * function will instantiate a pixel motion predictor with no parameters (defaults).
	 *
	 * Warning: motion prediction requires camera calibration to be set, otherwise
	 * the function will not instantiate the motion predictor.
	 * @param predictor				Pixel motion predictor instance.
	 */
	void setMotionPredictor(kinematics::PixelMotionPredictor::UniquePtr predictor) {
		if (predictor) {
			mPredictor = std::move(predictor);
		}
		else {
			if (mCamera) {
				mPredictor = std::make_unique<kinematics::PixelMotionPredictor>(mCamera);
			}
		}
	}

	/**
	 * Add scene depth, a median depth value of tracked landmarks usually works well enough.
	 * @param timedDepth 		Depth measurement value (pair of timestamp and measured depth)
	 */
	virtual void accept(const dv::measurements::Depth &timedDepth) {
		if (!mPredictor) {
			throw std::invalid_argument("Trying to add depth to a tracker that does not support motion awareness.");
		}
		mDepthHistory.insert(std::make_pair(timedDepth.mTimestamp, timedDepth.mDepth));

		// Retain only relevant depth measurements
		while ((mDepthHistory.rbegin()->first - mDepthHistory.begin()->first) > depthHistoryDuration) {
			mDepthHistory.erase(mDepthHistory.begin());
		}
	}

	/**
	 * Add camera transformation, usually in the world coordinate frame (`T_WC`). Although the
	 * class only extract the motion difference, so any other reference frame should also work as long as reference
	 * frames are not mixed up.
	 * @param transform 		Camera pose represented by a transformation
	 */
	virtual void accept(const dv::kinematics::Transformationf &transform) {
		if (!mPredictor) {
			throw std::invalid_argument(
				"Trying to add camera pose to a tracker that does not support motion awareness.");
		}
		mTransformer->pushTransformation(transform);
	}

	/**
	 * Check whether lookback is enabled.
	 * @return 	True if lookback rejection is enabled.
	 */
	[[nodiscard]] bool isLookbackRejectionEnabled() const {
		return mLookbackRejection;
	}

	/**
	 * Enable or disable lookback rejection based on Forward-Backward error. Lookback rejection applies Lucas-Kanade
	 * tracking backwards after running the usual tracking and rejects any tracks that fails to successfully track back
	 * to same approximate location by measuring Euclidean distance. Euclidean distance threshold for rejection can be
	 * set using `setRejectionDistanceThreshold` method.
	 *
	 * This is a real-time implementation of the method proposed by Zdenek et al. 2010, that only performs
	 * forward-backward error measurement within a single pair of latest and previous frame:
	 * http://kahlan.eps.surrey.ac.uk/featurespace/tld/Publications/2010_icpr.pdf
	 *
	 * @param lookbackRejection Pass true to enable lookback rejection based on Forward-Backward error.
	 * @sa setRejectionDistanceThreshold
	 */
	void setLookbackRejection(const bool lookbackRejection) {
		mLookbackRejection = lookbackRejection;
	}

	/**
	 * Get the current rejection distance threshold for the lookback rejection feature.
	 * @return	Rejection distance value which represents the Euclidean distance in pixel space between backward
	 * 			tracked feature pose and initial feature position before performing forward tracking.
	 * @sa setRejectionDistanceThreshold
	 */
	[[nodiscard]] float getRejectionDistanceThreshold() const {
		return mRejectionDistanceThreshold;
	}

	/**
	 * Set the threshold for lookback rejection feature. This value is a maximum Euclidean distance value that is
	 * considered successful when performing backwards tracking check after forward tracking. If the backward tracked
	 * feature location is further away from initial position than this given value, the tracker will reject the track
	 * as a failed track. See method `setLookbackRejection` documentation for further explanation of the approach.
	 * @param rejectionDistanceThreshold	Rejection distance threshold value.
	 * @sa setLookbackRejection
	 */
	void setRejectionDistanceThreshold(const float rejectionDistanceThreshold) {
		mRejectionDistanceThreshold = rejectionDistanceThreshold;
	}

	/**
	 * Get currently assumed constant depth value. It is used if no depth measurements are provided.
	 * @return 	Currently used aistance to the scene (depth).
	 * @sa setConstantDepth
	 */
	[[nodiscard]] float getConstantDepth() const {
		return constantDepth;
	}

	/**
	 * Set constant depth value that is assumed if no depth measurement is passed using
	 * `accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is just a
	 * reasonable guess.
	 *
	 * This value is used for predicting feature track positions when no depth measurements are passed in.
	 * @param depth 	Distance to the scene (depth).
	 * @throws InvalidArgument Exception is thrown if a negative depth value is passed.
	 */
	virtual void setConstantDepth(const float depth) {
		if (depth < 0.f) {
			throw dv::exceptions::InvalidArgument<float>(
				"Negative constant depth provided, please provide positive value only", depth);
		}

		constantDepth = depth;
	}
};

static_assert(concepts::Accepts<ImageFeatureLKTracker, dv::measurements::Depth>);
static_assert(concepts::Accepts<ImageFeatureLKTracker, dv::kinematics::Transformationf>);
static_assert(concepts::Accepts<ImageFeatureLKTracker, dv::Frame>);
static_assert(concepts::SupportsConstantDepth<ImageFeatureLKTracker>);

} // namespace dv::features

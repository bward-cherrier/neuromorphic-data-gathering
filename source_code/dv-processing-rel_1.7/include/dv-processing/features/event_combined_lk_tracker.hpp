#pragma once

#include "../core/core.hpp"
#include "../core/frame.hpp"
#include "../data/utilities.hpp"
#include "image_feature_lk_tracker.hpp"

namespace dv::features {

/**
 * Implements an event combined Lucas-Kanade tracker. The algorithms detects and tracks features
 * on a regular frame image, but to improve tracking quality, it accumulates intermediate
 * frames from events, performs tracking on those frames and uses the output to predict the
 * track locations on the regular frame.
 * @tparam AccumulatorType 	Accumulator class to be used for frame generation.
 */
template<dv::concepts::EventToFrameConverter<dv::EventStore> AccumulatorType = dv::EdgeMapAccumulator>
class EventCombinedLKTracker : public ImageFeatureLKTracker {
protected:
	std::unique_ptr<AccumulatorType> mAccumulator = nullptr;
	dv::Duration mStoreTimeLimit                  = dv::Duration(5000000);
	size_t mNumberOfEvents                        = 20000;
	double mMinRateForIntermediateTracking        = 0;
	int mNumIntermediateFrames                    = 3;
	dv::EventStore mEventBuffer;
	std::vector<dv::features::ImagePyramid> mAccumulatedFrames;

	std::vector<std::vector<cv::Point2f>> mEventTrackPoints;

	/**
	 * Run the intermediate tracking on accumulated events. The lastFrameResults are modified if any of
	 * the intermediate tracks are lost. The predicted coordinates are returned which must match the indices
	 * of the keypoints in lastFrameResults keypoint list.
	 * @return      Predicted feature track locations that correspond to modified `lastFrameResults->keypoints`
	 * 				vector.
	 */
	std::vector<cv::Point2f> trackIntermediateEvents() {
		mAccumulatedFrames.clear();
		mEventTrackPoints.clear();
		std::vector<cv::Point2f> prevPoints = dv::data::convertToCvPoints(lastFrameResults->keypoints);

		if (mEventBuffer.isEmpty()) {
			return prevPoints;
		}

		int64_t framePeriod        = mCurrentFrame->timestamp - lastFrameResults->timestamp;
		int64_t timestampIncrement = framePeriod / mNumIntermediateFrames;
		for (int64_t generationTime = lastFrameResults->timestamp; generationTime <= mCurrentFrame->timestamp;
			 generationTime         += timestampIncrement) {
			dv::EventStore slice
				= mEventBuffer.sliceTime(mEventBuffer.getLowestTime(), generationTime).sliceBack(mNumberOfEvents);
			mAccumulator->accept(slice);
			dv::Frame frame = mAccumulator->generateFrame();
			mAccumulatedFrames.emplace_back(
				generationTime, frame.image, mConfig.searchWindowSize, mConfig.numPyrLayers);
		}
		std::vector<cv::Point2f> currentPoints;
		std::vector<unsigned char> status;
		std::vector<float> err;
		auto prevFrame = mAccumulatedFrames.begin();
		for (auto nextFrame = mAccumulatedFrames.begin() + 1; nextFrame < mAccumulatedFrames.end();
			 nextFrame++, prevFrame++) {
			currentPoints = prevPoints;
			if (prevPoints.empty()) {
				// No more points to track
				continue;
			}
			mTracker->calc(prevFrame->pyramid, nextFrame->pyramid, prevPoints, currentPoints, status, err);
			for (size_t i = 0; i < status.size(); i++) {
				const auto &kpt = currentPoints[i];
				if (status[i] == 1 && mDetector->isWithinROI(kpt)) {
					// Replace prediction with newly tracked location
					prevPoints[i] = kpt;
				}
			}
			mEventTrackPoints.push_back(prevPoints);
		}
		dv::runtime_assert(lastFrameResults->keypoints.size() == prevPoints.size(),
			"Predicted points contains less features than the lastFrameResults");
		return prevPoints;
	}

	/**
	 * Perform the tracking.
	 * @return      Tracking result.
	 */
	Result::SharedPtr track() override {
		if (!mCurrentFrame) {
			return nullptr;
		}

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

		std::vector<cv::Point2f> predictedKpts;
		std::vector<cv::Point2f> previousKpts = dv::data::convertToCvPoints(lastFrameResults->keypoints);
		if (mEventBuffer.sliceTime(mPreviousFrame->timestamp, mCurrentFrame->timestamp).rate()
			> mMinRateForIntermediateTracking) {
			predictedKpts = trackIntermediateEvents();
		}
		else {
			predictedKpts = previousKpts;
		}

		mTracker->calc(mPreviousFrame->pyramid, mCurrentFrame->pyramid, previousKpts, predictedKpts, status, err);

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
					cv::circle(mask, cv::Point2f(feat.pt.x(), feat.pt.y()), static_cast<int>(std::ceil(feat.size)),
						cv::Scalar(0), -1, cv::LINE_4);
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
	 * Initialize the event combined Lucas-Kanade tracker and custom tracker parameters. It is going to use
	 * EdgeMapAccumulator with 15000 events and 0.25 event contribution. It will accumulate 3 intermediate
	 * frames from events to predict the track positions on regular frame.
	 * @param resolution        Image resolution.
	 * @param config            Image tracker configuration.
	 */
	EventCombinedLKTracker(const cv::Size &resolution, const ImageFeatureLKTracker::Config &config) :
		ImageFeatureLKTracker(resolution, config) {
		setAccumulator(nullptr);
	}

	/**
	 * Initialize the event combined Lucas-Kanade tracker and custom tracker parameters. It is going to use
	 * EdgeMapAccumulator with 15000 events and 0.25 event contribution. It will accumulate 3 intermediate
	 * frames from events to predict the track positions on regular frame.
	 * @param camera            Camera geometry.
	 * @param config            Image tracker configuration.
	 */
	EventCombinedLKTracker(
		const camera::CameraGeometry::SharedPtr &camera, const ImageFeatureLKTracker::Config &config) :
		ImageFeatureLKTracker(camera, config) {
		setAccumulator(nullptr);
	}

public:
	using SharedPtr = std::shared_ptr<EventCombinedLKTracker>;
	using UniquePtr = std::unique_ptr<EventCombinedLKTracker>;

	/**
	 * Create a tracker instance that performs tracking of features on both - event accumulated and regular images.
	 * Tracking is performed by detecting and tracking features on a regular image. It also uses events
	 * to generate intermediate accumulated frames between the regular frames, track the features on them
	 * and use the intermediate tracking results as feature position priors for the image frame.
	 * @param resolution 		Sensor resolution
	 * @param config 			Lucas-Kanade tracker configuration
	 * @param accumulator 		The accumulator instance to be used for intermediate frame accumulation.
	 * 							Uses `dv::EdgeMapAccumulator` with default parameters if `nullptr` is passed.
	 * @param detector 			Feature (corner) detector to be used.
	 * 							Uses `cv::Fast` with a threshold of 10 by default.
	 * @param redetection 		Feature redetection strategy.
	 * 							By default, redetects features when feature count is bellow 0.5 of maximum value.
	 * @return					The tracker instance
	 */
	[[nodiscard]] static EventCombinedLKTracker::UniquePtr RegularTracker(const cv::Size &resolution,
		const Config &config = Config(), std::unique_ptr<AccumulatorType> accumulator = nullptr,
		ImagePyrFeatureDetector::UniquePtr detector = nullptr, RedetectionStrategy::UniquePtr redetection = nullptr) {
		EventCombinedLKTracker::UniquePtr tracker{new EventCombinedLKTracker(resolution, config)};
		if (detector) {
			tracker->setDetector(std::move(detector));
		}
		if (redetection) {
			tracker->setRedetectionStrategy(std::move(redetection));
		}
		if (accumulator) {
			tracker->setAccumulator(std::move(accumulator));
		}
		return tracker;
	}

	/**
	 * Create a tracker instance that performs tracking of features on both - event accumulated and regular images.
	 * Tracking is performed by detecting and tracking features on a regular image. It also uses events
	 * to generate intermediate accumulated frames between the regular frames, track the features on them
	 * and use the intermediate tracking results as feature position priors for the image frame.
	 * The implementation also uses camera motion and scene depth to motion compensate events, so the intermediate
	 * accumulated frames are sharp and the Lucas-Kanade tracker works more accurately. This requires
	 * camera sensor to be calibrated.
	 * @param camera 			Camera geometry class instance, containing the intrinsic calibration of the
	 * 							camera sensor.
	 * @param config 			Lucas-Kanade tracker configuration
	 * @param accumulator 		The accumulator instance to be used for intermediate frame accumulation.
	 * 							Uses `dv::EdgeMapAccumulator` with default parameters if `nullptr` is passed.
	 * @param motionPredictor 	Motion predictor class, by default it uses pixel reprojection
	 * 							`dv::kinematics::PixelMotionPredictor` without distortion model.
	 * @param detector 			Feature (corner) detector to be used.
	 * 							Uses `cv::Fast` with a threshold of 10 by default.
	 * @param redetection 		Feature redetection strategy.
	 * 							By default, redetects features when feature count is bellow 0.5 of maximum value.
	 * @return					The tracker instance
	 */
	[[nodiscard]] static EventCombinedLKTracker::UniquePtr MotionAwareTracker(
		const camera::CameraGeometry::SharedPtr &camera, const Config &config = Config(),
		std::unique_ptr<AccumulatorType> accumulator                = nullptr,
		kinematics::PixelMotionPredictor::UniquePtr motionPredictor = nullptr,
		ImagePyrFeatureDetector::UniquePtr detector = nullptr, RedetectionStrategy::UniquePtr redetection = nullptr) {
		EventCombinedLKTracker::UniquePtr tracker{new EventCombinedLKTracker(camera, config)};
		if (detector) {
			tracker->setDetector(std::move(detector));
		}
		if (redetection) {
			tracker->setRedetectionStrategy(std::move(redetection));
		}
		if (accumulator) {
			tracker->setAccumulator(std::move(accumulator));
		}
		if (motionPredictor) {
			tracker->setMotionPredictor(std::move(motionPredictor));
		}
		else {
			tracker->setMotionPredictor(std::make_unique<kinematics::PixelMotionPredictor>(camera));
		}
		return tracker;
	}

	/**
	 * Add an event batch. Added events should contain at least some events that were registered further
	 * in the future of the next image.
	 * @param store         Batch of events.
	 */
	void accept(const dv::EventStore &store) {
		mEventBuffer.add(store);
		// Limit stored event count
		if (mEventBuffer.duration() > mStoreTimeLimit) {
			mEventBuffer.retainDuration(mStoreTimeLimit);
		}
	}

	/**
	 * Get the intermediate tracking points on the event frames.
	 * @return      A vector of tracked points on the intermediate frames.
	 */
	[[nodiscard]] const std::vector<std::vector<cv::Point2f>> &getEventTrackPoints() const {
		return mEventTrackPoints;
	}

	/**
	 * Get a vector containing the intermediate accumulated frames.
	 * @return      A vector containing the intermediate accumulated frames.
	 */
	[[nodiscard]] const std::vector<dv::features::ImagePyramid> &getAccumulatedFrames() const {
		return mAccumulatedFrames;
	}

	/**
	 * Get the event storage time limit.
	 * @return      Duration of the event storage in microseconds.
	 */
	[[nodiscard]] dv::Duration getStoreTimeLimit() const {
		return mStoreTimeLimit;
	}

	/**
	 * Set the event buffer storage duration limit.
	 * @param storeTimeLimit       Storage duration limit in microseconds.
	 */
	void setStoreTimeLimit(const dv::Duration storeTimeLimit) {
		mStoreTimeLimit = storeTimeLimit;
	}

	/**
	 * Get the number of latest events that are going to be accumulated for each frame.
	 * @return      Number of accumulated events.
	 */
	[[nodiscard]] size_t getNumberOfEvents() const {
		return mNumberOfEvents;
	}

	/**
	 * Set the number of latest events that are going to be accumulated for each frame.
	 * @param _numberOfEvents       Number of accumulated events.
	 */
	void setNumberOfEvents(const size_t numberOfEvents) {
		mNumberOfEvents = numberOfEvents;
	}

	/**
	 * Get the number of intermediate frames that are going to be generated.
	 * @return      Number of intermediate frames between the frames.
	 */
	[[nodiscard]] int getNumIntermediateFrames() const {
		return mNumIntermediateFrames;
	}

	/**
	 * Set the number of intermediate frames that are going to be generated.
	 * @param numIntermediateFrames     Number of intermediate frames between the frames.
	 */
	void setNumIntermediateFrames(const int numIntermediateFrames) {
		mNumIntermediateFrames = numIntermediateFrames;
	}

	/**
	 * Set an accumulator instance to be used for frame generation. If a `nullptr` is passed, the
	 * function will instantiate an accumulator with no parameters (defaults).
	 * @param accumulator		An accumulator instance, can be nullptr to instantiate a default accumulator.
	 */
	void setAccumulator(std::unique_ptr<AccumulatorType> accumulator) {
		if (accumulator) {
			mAccumulator = std::move(accumulator);
		}
		else {
			mAccumulator = std::make_unique<AccumulatorType>(mResolution);
		}
	}

	/**
	 * Add scene depth, a median depth value of tracked landmarks usually works well enough.
	 * @param timedDepth 		Depth measurement value (pair of timestamp and measured depth)
	 */
	void accept(const dv::measurements::Depth &timedDepth) override {
		if constexpr (concepts::Accepts<AccumulatorType, dv::measurements::Depth>) {
			mAccumulator->accept(timedDepth);
		}
		ImageFeatureLKTracker::accept(timedDepth);
	}

	/**
	 * Add camera transformation, usually in the world coordinate frame (`T_WC`). Although the
	 * class only extract the motion difference, so any other reference frame should also work as long as reference
	 * frames are not mixed up.
	 * @param transform 		Camera pose represented by a transformation.
	 */
	void accept(const kinematics::Transformationf &transform) override {
		if constexpr (concepts::Accepts<AccumulatorType, kinematics::Transformationf>) {
			mAccumulator->accept(transform);
		}
		ImageFeatureLKTracker::accept(transform);
	}

	/**
	 * Add an input image for the tracker. Image pyramid will be built from the given image.
	 * @param image             Acquired image.
	 */
	void accept(const dv::Frame &image) override {
		ImageFeatureLKTracker::accept(image);
	}

	/**
	 * Get the minimum event rate that is required to perform intermediate tracking.
	 * @return 		Minimum event rate per second value.
	 */
	[[nodiscard]] double getMinRateForIntermediateTracking() const {
		return mMinRateForIntermediateTracking;
	}

	/**
	 * Set a minimum event rate per second value that is used to perform intermediate. If the event rate between
	 * last and current frame is lower than this, tracker assumes very little motion and does not perform
	 * intermediate tracking.
	 * @param minRateForIntermediateTracking	Event rate (number of incoming events per second) required
	 * 											to perform intermediate tracking on accumulated frames.
	 */
	void setMinRateForIntermediateTracking(const double minRateForIntermediateTracking) {
		mMinRateForIntermediateTracking = minRateForIntermediateTracking;
	}

	/**
	 * Set constant depth value that is assumed if no depth measurement is passed using
	 * `accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is just a
	 * reasonable guess.
	 *
	 * This value is propagated into the accumulator if it supports constant depth setting.
	 * @param depth 	Distance to the scene (depth).
	 * @throws InvalidArgument Exception is thrown if a negative depth value is passed.
	 */
	void setConstantDepth(const float depth) override {
		ImageFeatureLKTracker::setConstantDepth(depth);
		if constexpr (concepts::SupportsConstantDepth<AccumulatorType>) {
			mAccumulator->setConstantDepth(depth);
		}
	}
};

static_assert(concepts::Accepts<EventCombinedLKTracker<>, dv::measurements::Depth>);
static_assert(concepts::Accepts<EventCombinedLKTracker<>, dv::kinematics::Transformationf>);
static_assert(concepts::Accepts<EventCombinedLKTracker<>, dv::EventStore>);
static_assert(concepts::Accepts<EventCombinedLKTracker<>, dv::Frame>);
static_assert(concepts::SupportsConstantDepth<EventCombinedLKTracker<>>);

} // namespace dv::features

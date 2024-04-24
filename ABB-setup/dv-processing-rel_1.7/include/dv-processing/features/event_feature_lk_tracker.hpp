#pragma once

#include "../core/frame.hpp"
#include "image_feature_lk_tracker.hpp"

namespace dv::features {
/**
 * Event-based Lucas-Kanade tracker, the tracking is achieved by accumulating frames and running
 * the classic LK frame based tracker on them.
 *
 * Since the batch of events might contain information for more than a single tracking iteration
 * configurable by the framerate parameter, the tracking function should be executed on a loop
 * until it returns a null-pointer, signifying end of available data processing:
 * ```
 * tracker.addEventInput(eventStore);
 * while (auto result = tracker.runTracking()) {
 *     // process the tracking result
 * }
 *```
 * @tparam AccumulatorType 	Accumulator class to be used for frame generation.
 */
template<concepts::EventToFrameConverter<dv::EventStore> AccumulatorType = dv::EdgeMapAccumulator>
class EventFeatureLKTracker : public ImageFeatureLKTracker {
private:
	// This hide the accept, overloads are available or necessary types
	using ImageFeatureLKTracker::accept;

protected:
	std::unique_ptr<AccumulatorType> mAccumulator = nullptr;
	int mFramerate                                = 50;
	int64_t mPeriod                               = 1000000 / mFramerate;
	int64_t mLastRunTimestamp                     = 0;
	dv::Duration mStoreTimeLimit                  = dv::Duration(5000000);

	/**
	 * The default number of event is a third of of total pixels in the sensor.
	 */
	size_t mNumberOfEvents;
	dv::EventStore mEventBuffer;
	cv::Mat mAccumulatedFrame;

	/**
	 * Perform the tracking
	 * @return      Tracking result.
	 */
	Result::SharedPtr track() override {
		if (mEventBuffer.size() < mNumberOfEvents) {
			return nullptr;
		}
		if (mLastRunTimestamp == 0) {
			mLastRunTimestamp = mEventBuffer.slice(mNumberOfEvents - 1, 1).getLowestTime() - mPeriod;
		}
		int64_t nextRunTimestamp = mLastRunTimestamp + mPeriod;
		if (nextRunTimestamp <= mEventBuffer.getHighestTime()) {
			dv::EventStore slice
				= mEventBuffer.sliceTime(mEventBuffer.getLowestTime(), nextRunTimestamp + 1).sliceBack(mNumberOfEvents);
			mAccumulator->accept(slice);
			dv::Frame frame   = mAccumulator->generateFrame();
			mAccumulatedFrame = frame.image;
			accept(dv::Frame(nextRunTimestamp, mAccumulatedFrame));
			mLastRunTimestamp = nextRunTimestamp;
			return ImageFeatureLKTracker::track();
		}
		else {
			return nullptr;
		}
	}

	/**
	 * Initialize the event-frame tracker with default configuration: all the defaults of ImageFeatureLKTracker
	 * and a EdgeMapAccumulator executing at 50 FPS with and event count equal to third of the camera resolution and
	 * event contribution of 0.25.
	 * @param imageDimensions   Image resolution.
	 * @param config   			Lukas-Kanade tracker configuration.
	 */
	explicit EventFeatureLKTracker(const cv::Size &dimensions, const Config &config) :
		ImageFeatureLKTracker(dimensions, config),
		mNumberOfEvents(mResolution.area() / 3) {
		setAccumulator(nullptr);
	}

	/**
	 * Initialize the event-frame tracker with default configuration: all the defaults of ImageFeatureLKTracker
	 * and a EdgeMapAccumulator executing at 50 FPS with and event count equal to third of the camera resolution and
	 * event contribution of 0.25.
	 * @param camera   			Camera geometry.
	 * @param config   			Lukas-Kanade tracker configuration.
	 */
	explicit EventFeatureLKTracker(const dv::camera::CameraGeometry::SharedPtr &camera, const Config &config) :
		ImageFeatureLKTracker(camera, config),
		mNumberOfEvents(mResolution.area() / 3) {
		setAccumulator(nullptr);
	}

public:
	using SharedPtr = std::shared_ptr<EventFeatureLKTracker>;
	using UniquePtr = std::unique_ptr<EventFeatureLKTracker>;

	/**
	 * Create a tracker instance that performs tracking of features on event accumulated frames.
	 * Features are detected and tracked on event accumulated frames.
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
	[[nodiscard]] static EventFeatureLKTracker::UniquePtr RegularTracker(const cv::Size &resolution,
		const Config &config = Config(), std::unique_ptr<AccumulatorType> accumulator = nullptr,
		ImagePyrFeatureDetector::UniquePtr detector = nullptr, RedetectionStrategy::UniquePtr redetection = nullptr) {
		EventFeatureLKTracker::UniquePtr tracker{new EventFeatureLKTracker(resolution, config)};
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
	 * Create a tracker instance that performs tracking of features on event accumulated frames.
	 * Features are detected and tracked on event accumulated frames. Additionally, camera motion and scene depth
	 * are used to generate motion compensated frames, which are way sharper than usual accumulated frames. This
	 * requires camera sensor to be calibrated.
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
	[[nodiscard]] static EventFeatureLKTracker::UniquePtr MotionAwareTracker(
		const camera::CameraGeometry::SharedPtr &camera, const Config &config = Config(),
		std::unique_ptr<AccumulatorType> accumulator                = nullptr,
		kinematics::PixelMotionPredictor::UniquePtr motionPredictor = nullptr,
		ImagePyrFeatureDetector::UniquePtr detector = nullptr, RedetectionStrategy::UniquePtr redetection = nullptr) {
		EventFeatureLKTracker::UniquePtr tracker{new EventFeatureLKTracker(camera, config)};
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
	 * Get the latest accumulated frame.
	 * @return      An accumulated frame.
	 */
	[[nodiscard]] const cv::Mat &getAccumulatedFrame() const {
		return mAccumulatedFrame;
	}

	/**
	 * Get configured framerate.
	 * @return      Current accumulation and tracking framerate.
	 */
	[[nodiscard]] int getFramerate() const {
		return mFramerate;
	}

	/**
	 * Set the accumulation and tracking framerate.
	 * @param framerate_        New accumulation and tracking framerate.
	 */
	void setFramerate(int framerate) {
		mFramerate = framerate;
		mPeriod    = 1000000 / framerate;
	}

	/**
	 * Add the input events.
	 * Since the batch of events might contain information for more than a single tracking iteration
	 * configurable by the framerate parameter, the tracking function should be executed on a loop
	 * until it returns a null-pointer, signifying end of available data processing:
	 * ```
	 * tracker.addEventInput(eventStore);
	 * while (auto result = tracker.runTracking()) {
	 *     // process the tracking result
	 * }
	 *```
	 * @param store         Event batch.
	 */
	void accept(const dv::EventStore &store) {
		mEventBuffer.add(store);
		// Limit stored event count
		if (mEventBuffer.duration() > mStoreTimeLimit) {
			auto sliced = mEventBuffer.sliceTime(
				mEventBuffer.getHighestTime() - mStoreTimeLimit.count(), mEventBuffer.getHighestTime() + 1);
			if (sliced.size() >= mNumberOfEvents) {
				mEventBuffer = sliced;
			}
		}
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
	 * The default number of event is a third of of total pixels in the sensor.
	 * @return      Number of event to be accumulated.
	 */
	[[nodiscard]] size_t getNumberOfEvents() const {
		return mNumberOfEvents;
	}

	/**
	 * Set the number of latest events that are going to be accumulated for each frame.
	 * The default number of event is a third of of total pixels in the sensor.
	 * @param numberOfEvents    	Number of accumulated events.
	 */
	void setNumberOfEvents(size_t numberOfEvents) {
		mNumberOfEvents = numberOfEvents;
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
	 * Set constant depth value that is assumed if no depth measurement is passed using
	 * `accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is just a
	 * reasonable guess.
	 *
	 * This value is used for predicting feature track positions when no depth measurements are passed in and also
	 * is propagated into the accumulator if it supports constant depth setting.
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

static_assert(concepts::Accepts<EventFeatureLKTracker<>, dv::measurements::Depth>);
static_assert(concepts::Accepts<EventFeatureLKTracker<>, dv::kinematics::Transformationf>);
static_assert(concepts::Accepts<EventFeatureLKTracker<>, dv::EventStore>);
static_assert(concepts::SupportsConstantDepth<EventFeatureLKTracker<>>);

} // namespace dv::features

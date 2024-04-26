#pragma once

#include "tracker_base.hpp"

namespace dv::features {

/**
 * Implementation of different redetection strategies for trackers.
 */
class RedetectionStrategy {
public:
	typedef std::shared_ptr<RedetectionStrategy> SharedPtr;
	typedef std::unique_ptr<RedetectionStrategy> UniquePtr;

	/**
	 * Decide the redetection of tracker features depending on the state of the tracker.
	 * @param tracker       Current state of the tracker.
	 * @return              True to perform redetection of features, false to continue.
	 */
	[[nodiscard]] virtual bool decideRedetection(const dv::features::TrackerBase &tracker) = 0;

	/**
	 * Decide the redetection of tracker features depending on the state of the tracker.
	 * @param tracker       Current state of the tracker.
	 * @return              True to perform redetection of features, false to continue.
	 * @deprecated Use decideRedetection instead
	 */
	[[deprecated("Use decideRedetection instead")]] [[nodiscard]] bool decideRedection(
		const dv::features::TrackerBase &tracker) {
		return decideRedetection(tracker);
	}

	virtual ~RedetectionStrategy() = default;
};

/**
 * No redetection strategy.
 */
class NoRedetection : public RedetectionStrategy {
public:
	/**
	 * Do not perform redetection.
	 * @return          Just return false always.
	 */
	bool decideRedetection(const TrackerBase &) override {
		return false;
	}
};

/**
 * Redetection strategy based on number of features.
 */
class FeatureCountRedetection : public RedetectionStrategy {
public:
	/**
	 * Redetection strategy based on number of features.
	 * @param minimumProportionOfTracks     Feature count coefficient, redetection is performed
	 *                              when feature count goes lower than the given proportion
	 *                              of maximum tracks, redetection will be executed.
	 */
	explicit FeatureCountRedetection(float minimumProportionOfTracks) :
		mMinimumProportionOfTracks(minimumProportionOfTracks) {
	}

	/**
	 * Check whether to perform redetection.
	 * @param tracker       Current state of the tracker.
	 * @return              True to perform redetection of features, false to continue.
	 */
	[[nodiscard]] bool decideRedetection(const TrackerBase &tracker) override {
		return static_cast<float>(tracker.getLastFrameResults()->keypoints.size())
			 < (static_cast<float>(tracker.getMaxTracks()) * mMinimumProportionOfTracks);
	}

protected:
	float mMinimumProportionOfTracks = 0.5f;
};

/**
 * Redetection strategy based on interval from last detection.
 */
class UpdateIntervalRedetection : public RedetectionStrategy {
public:
	/**
	 * Redetection strategy based on updating if specific amount of time from last detection has passed.
	 */
	explicit UpdateIntervalRedetection(const dv::Duration updateInterval) :
		mUpdateIntervalTime(updateInterval.count()) {
	}

	/**
	 * Check whether to perform redetection.
	 */
	[[nodiscard]] bool decideRedetection(const TrackerBase &tracker) override {
		return (tracker.getLastFrameResults()->timestamp - mLastDetectionTime) > mUpdateIntervalTime;
	}

protected:
	const int64_t mUpdateIntervalTime;
	int64_t mLastDetectionTime = -std::numeric_limits<int64_t>::infinity();
};

/**
 * Redetection strategy based on interval from last detection or minimum number of tracks.
 * This class combines redetection logic from UpdateIntervalRedetection and FeatureCountRedetection.
 */
class UpdateIntervalOrFeatureCountRedetection : public RedetectionStrategy {
public:
	/**
	 * Redetection strategy based on updating if specific amount of time from last detection has passed  or minimum
	 * number of tracks to follow.
	 */
	explicit UpdateIntervalOrFeatureCountRedetection(
		const dv::Duration updateInterval, const float minimumProportionOfTracks) :
		updateIntervalRedetection(updateInterval),
		featureCountRedetection(minimumProportionOfTracks) {
	}

	/**
	 * Check whether to perform redetection.
	 */
	[[nodiscard]] bool decideRedetection(const TrackerBase &tracker) override {
		return updateIntervalRedetection.decideRedetection(tracker)
			|| featureCountRedetection.decideRedetection(tracker);
	}

private:
	UpdateIntervalRedetection updateIntervalRedetection;
	FeatureCountRedetection featureCountRedetection;
};

} // namespace dv::features

#pragma once

#include "feature_detector.hpp"

namespace dv::features {

/**
 * A base class for implementing feature trackers, that track sets of features against
 * streams of various inputs. This class specifically does not define an input type,
 * so it could be defined by the specific implementation.
 */
class TrackerBase {
public:
	/**
	 * Result of tracking.
	 */
	struct Result {
		/**
		 * A vector of keypoints.
		 */
		dv::cvector<dv::TimedKeyPoint> keypoints = {};

		/**
		 * A flag that notifies the user of the tracker that this
		 * specific input caused redetection to happen and the tracker not
		 * only tracked the buffered events, but also detected new features.
		 */
		bool asKeyFrame = false;

		/**
		 * Timestamp of the execution, it can be frame timestamp or last timestamp of an event slice.
		 */
		int64_t timestamp = 0;

		typedef std::shared_ptr<Result> SharedPtr;

		typedef std::shared_ptr<const Result> ConstPtr;

		/**
		 * Construct tracking result
		 * @param _timestamp        Execution time of tracking
		 * @param _keypoints        The resulting features
		 * @param keyframe          Whether this set of features can be regarded as
		 *                          a keyframe (redetection was triggered)
		 */
		Result(const int64_t _timestamp, const dv::cvector<dv::TimedKeyPoint> &_keypoints, const bool keyframe) :
			keypoints(_keypoints),
			asKeyFrame(keyframe),
			timestamp(_timestamp) {
		}

		Result() = default;
	};

	typedef std::shared_ptr<TrackerBase> SharedPtr;
	typedef std::unique_ptr<TrackerBase> UniquePtr;

protected:
	/**
	 * Maximum number of tracks.
	 */
	size_t maxTracks = 200;

	/**
	 * Cached results of last tracker execution.
	 */
	Result::SharedPtr lastFrameResults;

	/**
	 * Virtual function that is called after all inputs were set.
	 * This function should perform tracking against `lastFrameResults`.
	 * @return      Tracking result.
	 */
	[[nodiscard]] virtual Result::SharedPtr track() = 0;

public:
	/**
	 * Set the maximum number of tracks.
	 * @param _maxTracks	Maximum number of tracks
	 */
	void setMaxTracks(size_t _maxTracks) {
		TrackerBase::maxTracks = _maxTracks;
	}

	/**
	 * Get the maximum number of tracks.
	 * @return				Maximum number of tracks
	 */
	[[nodiscard]] size_t getMaxTracks() const {
		return maxTracks;
	}

	/**
	 * Retrieve cached last frame detection results.
	 * @return		Detection result from the last processed frame.
	 */
	[[nodiscard]] const Result::SharedPtr &getLastFrameResults() const {
		return lastFrameResults;
	}

	/**
	 * Performed the tracking and cache the results.
	 * @return      Tracking result.
	 */
	[[nodiscard]] Result::ConstPtr runTracking() {
		auto result = track();
		if (result) {
			lastFrameResults = result;
		}
		return result;
	}

	virtual ~TrackerBase() = default;

	/**
	 * Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value
	 * of the keypoint structure.
	 * @param trackIds      Track `class_id` values to be removed from cached tracker results.
	 */
	virtual void removeTracks(const std::vector<int> &trackIds) {
		for (int id : trackIds) {
			auto iter = lastFrameResults->keypoints.begin();
			while (iter != lastFrameResults->keypoints.end()) {
				if (iter->class_id == id) {
					iter = lastFrameResults->keypoints.erase(iter);
				}
				else {
					iter++;
				}
			}
		}
	}
};

} // namespace dv::features

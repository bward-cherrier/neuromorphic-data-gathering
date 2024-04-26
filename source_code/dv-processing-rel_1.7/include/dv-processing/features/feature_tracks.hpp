#pragma once

#include "../core/utils.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "../visualization/colors.hpp"
#include "tracker_base.hpp"

namespace dv::features {

/**
 * A class to store a time limited amount of feature tracks. Sorts and stores the data in separate queues for
 * each track id. Provides `visualize` function to generate visualization images of the tracks.
 */
class FeatureTracks {
private:
	std::map<int32_t, std::shared_ptr<std::deque<dv::TimedKeyPoint>>> mHistory;

	dv::Duration mHistoryDuration = dv::Duration(500'000);

	std::optional<dv::Duration> mTrackTimeout = std::nullopt;

	int64_t mHighestTime = -1;

	/**
	 * Add a keypoint measurement
	 * @param keypoint 		Keypoint measurement
	 */
	void addKeypoint(const dv::TimedKeyPoint &keypoint) {
		if (!mHistory.contains(keypoint.class_id)) {
			mHistory.emplace(keypoint.class_id, std::make_shared<std::deque<dv::TimedKeyPoint>>());
		}
		auto &history = mHistory[keypoint.class_id];
		history->push_back(keypoint);

		if (keypoint.timestamp > mHighestTime) {
			mHighestTime = keypoint.timestamp;
		}
	}

	/**
	 * Check the whole buffer for out-of-limit data, remove any tracks that do not contain any measurements.
	 */
	void maintainBufferDuration() {
		if (mTrackTimeout.has_value()) {
			std::erase_if(mHistory, [this](const auto &track) {
				return !track.second->empty()
					&& dv::Duration(mHighestTime - track.second->back().timestamp) > mTrackTimeout;
			});
		}

		// Maintain history duration
		for (auto &[_, track] : mHistory) {
			while (!track->empty() && dv::Duration(mHighestTime - track->front().timestamp) > mHistoryDuration) {
				track->pop_front();
			}
		}
		std::erase_if(mHistory, [](const auto &track) {
			return track.second->empty();
		});
	}

public:
	/**
	 * Add a keypoint measurement into the feature track.
	 * @param keypoint 		Single keypoint measurement.
	 */
	void accept(const dv::TimedKeyPoint &keypoint) {
		addKeypoint(keypoint);
		maintainBufferDuration();
	}

	/**
	 * Add a set of keypoint measurements into the feature track.
	 * @param keypoints 		Vector of keypoint measurements.
	 */
	void accept(const dv::TimedKeyPointPacket &keypoints) {
		for (const auto &keypoint : keypoints.elements) {
			addKeypoint(keypoint);
		}
		maintainBufferDuration();
	}

	/**
	 * Add OpenCV type keypoint. It is missing a timestamp, so current system clock time will be used for the timestamp.
	 * @param keypoint		KeyPoint measurement.
	 */
	void accept(const cv::KeyPoint &keypoint) {
		accept(dv::TimedKeyPoint(dv::Point2f(keypoint.pt.x, keypoint.pt.y), keypoint.size, keypoint.angle,
			keypoint.response, keypoint.octave, keypoint.class_id, dv::now()));
	}

	/**
	 * Add keypoint tracking result from a tracker.
	 * @param trackingResult 	Tracking results.
	 */
	void accept(const TrackerBase::Result::ConstPtr &trackingResult) {
		for (const auto &kp : trackingResult->keypoints) {
			addKeypoint(kp);
		}
		maintainBufferDuration();
	}

	/**
	 * Retrieve the history duration.
	 * @return 		Currently applied track history time limit.
	 */
	[[nodiscard]] Duration getHistoryDuration() const {
		return mHistoryDuration;
	}

	/**
	 * Set new history duration limit to buffer. If the new limit is shorter than the previously set,
	 * the tracks will be reduced to the new limit right away.
	 * @param historyDuration 	New time limit for the track history buffer.
	 */
	void setHistoryDuration(const dv::Duration historyDuration) {
		static const dv::Duration zeroDuration(0);
		if (historyDuration <= zeroDuration) {
			throw dv::exceptions::InvalidArgument<dv::Duration>(
				"Track history duration for the FeatureTracks must be positive non-zero duration value.");
		}
		mHistoryDuration = historyDuration;
		maintainBufferDuration();
	}

	/**
	 * Retrieve a track of given track id.
	 * @param trackId 		Track id to retrieve.
	 * @return 				A pointer to feature track history, `std::nullopt` if unavailable.
	 */
	[[nodiscard]] std::optional<std::shared_ptr<const std::deque<dv::TimedKeyPoint>>> getTrack(
		const int32_t trackId) const {
		if (mHistory.contains(trackId)) {
			return std::static_pointer_cast<const std::deque<dv::TimedKeyPoint>>(mHistory.at(trackId));
		}
		return std::nullopt;
	}

	/**
	 * Return all track ids that are available in the buffer.
	 * @return 		A vector containing track ids store in the history buffer.
	 */
	[[nodiscard]] std::vector<int32_t> getTrackIds() const {
		std::vector<int32_t> keys;
		keys.reserve(mHistory.size());
		for (const auto &[id, _] : mHistory) {
			keys.push_back(id);
		}
		return keys;
	}

	/**
	 * Return last keypoint from all tracks in the history.
	 * @return
	 */
	dv::TimedKeyPointPacket getLatestTrackKeypoints() {
		dv::TimedKeyPointPacket lastKeypoints;
		for (const auto &[_, track] : mHistory) {
			lastKeypoints.elements.push_back(track->back());
		}

		return lastKeypoints;
	}

	/**
	 * Run a callback function to each of the stored tracks.
	 * @param callback 		Callback function that is going to be called for each of the tracks, tracks are
	 * 						passed into the callback function as arguments.
	 */
	void eachTrack(
		const std::function<void(const int32_t, const std::shared_ptr<const std::deque<dv::TimedKeyPoint>> &)>
			&callback) const {
		for (const auto &[id, history] : mHistory) {
			callback(id, std::static_pointer_cast<const std::deque<dv::TimedKeyPoint>>(history));
		}
	}

	/**
	 * Draws tracks on the input image, by default uses neon color palette from the `dv::visualization::colors`
	 * namespace for each of the tracks.
	 * @param background        Background image to be used for tracks.
	 * @return 					Input image with drawn colored feature tracks.
	 * @throws InvalidArgument	An `InvalidArgument` exception is thrown if an empty image is passed as background.
	 */
	[[nodiscard]] cv::Mat visualize(const cv::Mat &background) const {
		cv::Mat output;
		if (background.channels() != 3) {
			cv::cvtColor(background, output, cv::COLOR_GRAY2BGR);
		}
		else {
			background.copyTo(output);
		}

		if (output.empty()) {
			throw dv::exceptions::InvalidArgument<void *>(
				"Empty image was passed into the FeatureTracks for visualization.");
		}

		for (const auto &[id, history] : mHistory) {
			auto iter        = history->crbegin();
			const auto color = dv::visualization::colors::someNeonColor(id);
			cv::Point2f prevPoint;
			while (iter != history->crend()) {
				if (iter == history->crbegin()) {
					// Just draw the marker
					prevPoint = cv::Point2f(iter->pt.x(), iter->pt.y());
					cv::drawMarker(output, prevPoint, color, cv::MARKER_SQUARE, 5);
					iter++;
					continue;
				}
				cv::Point2f currPoint = cv::Point2f(iter->pt.x(), iter->pt.y());
				cv::line(output, prevPoint, currPoint, color, 1);
				prevPoint = std::move(currPoint);
				iter++;
			}
		}
		return output;
	}

	/**
	 * Checks whether the feature track history buffer is empty.
	 * @return 		True if there are no feature keypoints in the buffer.
	 */
	[[nodiscard]] bool isEmpty() const {
		if (mHistory.empty()) {
			return true;
		}

		return !std::any_of(mHistory.begin(), mHistory.end(), [](const auto &track) {
			return !track.second->empty();
		});
	}

	/**
	 * Deletes any data stored in feature track buffer and resets visualization image.
	 */
	void clear() {
		mHistory.clear();
		mHighestTime = -1;
	}

	/**
	 * Get the track timeout value.
	 * @return Current track timeout value.
	 * @sa setTrackTimeout
	 */
	[[nodiscard]] const std::optional<dv::Duration> &getTrackTimeout() const {
		return mTrackTimeout;
	}

	/**
	 * Set the track timeout value, pass `std::nullopt` to disable the this feature at all. Track latest timestamp is
	 * going to be compared to highest received timestamp in accept method, if the value is exceeded the track is going
	 * to be removed. This is useful to remove lost tracks without waiting for the track history to remove it, consider
	 * setting it to 2x of tracking rate, so tracks will remove if the track is not updated for two consecutive frames.
	 *
	 * By default the feature is disabled, so lost tracks are kept until it's removed by the history time limit.
	 * @param trackTimeout 	Track timeout value or `std::nullopt` to disable the feature.
	 */
	void setTrackTimeout(const std::optional<dv::Duration> &trackTimeout) {
		mTrackTimeout = trackTimeout;
	}

	/**
	 * Return latest time from all existing tracks.
	 */
	int64_t getHighestTime() {
		return mHighestTime;
	}
};

static_assert(concepts::Accepts<FeatureTracks, dv::TimedKeyPoint>);
static_assert(concepts::Accepts<FeatureTracks, dv::TimedKeyPointPacket>);
static_assert(concepts::Accepts<FeatureTracks, cv::KeyPoint>);
static_assert(concepts::Accepts<FeatureTracks, TrackerBase::Result::ConstPtr>);

} // namespace dv::features

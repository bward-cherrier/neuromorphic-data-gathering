#pragma once

#include "core.hpp"

namespace dv {

template<class EventStoreType>
class AddressableStereoEventStreamSlicer {
protected:
	std::optional<size_t> minimumEvents     = std::nullopt;
	std::optional<dv::Duration> minimumTime = std::nullopt;
	StreamSlicer<EventStoreType> slicer;
	EventStoreType leftEvents;
	EventStoreType rightEvents;
	int64_t rightEventSeek = -1;

	/**
	 * Perform book-keeping of the right camera buffer by retaining data from a given timestamp. Events are
	 * "forgot" only if minimum amount and time duration values are maintained according to slicing configuration.
	 * @param timestampFrom 	Perform book-keeping by retaining data from this timestamp onward.
	 */
	void clearRightEventsBuffer(const int64_t timestampFrom) {
		auto store = rightEvents.sliceTime(timestampFrom);
		if ((!minimumTime.has_value() || store.duration() >= minimumTime)
			&& (!minimumEvents.has_value() || store.size() >= minimumEvents)) {
			rightEvents = store;
		}
	}

public:
	/**
	 * Adds EventStores from the left and right camera. Performs job evaluation immediately.
	 * @param leftEvents the EventStore from left camera.
	 * @param rightEvents the EventStore from right camera.
	 */
	void accept(const std::optional<EventStoreType> &left, const std::optional<EventStoreType> &right) {
		// Buffer events
		if (right.has_value()) {
			rightEvents.add(*right);
		}
		if (left.has_value()) {
			leftEvents.add(*left);
		}

		const auto syncedEvents = leftEvents.sliceTime(leftEvents.getLowestTime(), rightEvents.getHighestTime() + 1);
		if (syncedEvents.isEmpty()) {
			return;
		}

		// Add left and evaluate jobs
		slicer.accept(syncedEvents);

		leftEvents = leftEvents.sliceTime(rightEvents.getHighestTime() + 1, leftEvents.getHighestTime() + 1);

		if (rightEventSeek > 0) {
			clearRightEventsBuffer(rightEventSeek + 1);
		}
	}

	/**
	 * Perform an action on the stereo stream data every given amount of events.
	 * Event count is evaluated on the left camera stream and according time interval of data
	 * is sliced from the right camera event stream.
	 * Sliced data is passed into the callback function as soon as it arrived, first
	 * argument is left camera events and second is right camera events.
	 * Since right camera events are sliced by the time interval of left camera, the
	 * amount of events on right camera can be different.
	 * @param n 		the interval (in number of events) in which the callback should be called.
	 * @param callback 	the callback function that gets called on the data every interval.
	 * @return			Job identifier
	 * @sa AddressableEventStreamSlicer::doEveryNumberOfEvents
	 */
	int doEveryNumberOfEvents(
		const size_t n, std::function<void(const EventStoreType &, const EventStoreType &)> callback) {
		if (!minimumEvents.has_value() || minimumEvents < n) {
			minimumEvents = n;
		}

		return slicer.doEveryNumberOfElements(n, [this, callback](const EventStoreType &left) {
			const auto rightSlice = rightEvents.sliceTime(left.getLowestTime(), left.getHighestTime() + 1);
			callback(left, rightSlice);
			rightEventSeek = rightSlice.getHighestTime();
		});
	}

	/**
	 * Perform an action on the stereo stream data every given time interval.
	 * Event period is evaluated on the left camera stream and according time interval of data
	 * is sliced from the right camera event stream.
	 * Sliced data is passed into the callback function as soon as it arrived, first
	 * argument is left camera events and second is right camera events.
	 * @param interval 	Time interval to call the callback function. The callback is called
	 * 					based on timestamps of left camera.
	 * @param callback 	Function to be executed
	 * @return 			Job identifier.
	 * @sa AddressableEventStreamSlicer::doEveryTimeInterval
	 */
	int doEveryTimeInterval(
		const dv::Duration interval, std::function<void(const EventStoreType &, const EventStoreType &)> callback) {
		if (!minimumTime.has_value() || minimumTime < interval) {
			minimumTime = interval;
		}

		return slicer.doEveryTimeInterval(interval, [this, callback](const EventStoreType &left) {
			const auto rightSlice = rightEvents.sliceTime(left.getLowestTime(), left.getHighestTime() + 1);
			callback(left, rightSlice);
			rightEventSeek = rightSlice.getHighestTime();
		});
	}

	/**
	 * Returns true if the slicer contains the slicejob with the provided id
	 * @param job the id of the slicejob in question
	 * @return true, if the slicer contains the given slicejob
	 */
	bool hasJob(const int job) {
		return slicer.hasJob(job);
	}

	/**
	 * Removes the given job from the list of current jobs.
	 * @param job The job id to be removed
	 */
	void removeJob(const int job) {
		slicer.removeJob(job);
	}
};

using StereoEventStreamSlicer = AddressableStereoEventStreamSlicer<dv::EventStore>;

} // namespace dv

#pragma once

#include "../exception/exceptions/generic_exceptions.hpp"
#include "concepts.hpp"
#include "time_window.hpp"
#include "utils.hpp"

#include <functional>
#include <map>

namespace dv {

namespace concepts {
/**
 * Concept that verifies that given type is compatible for use with stream slicer.
 * @tparam Type Type to verify
 */
template<class Type>
concept CompatibleWithSlicer = requires {
								   requires dv::concepts::EventStorage<Type> || dv::concepts::DataPacket<Type>
												|| dv::concepts::TimestampedIterable<Type>;
							   };
} // namespace concepts

/**
 * The StreamSlicer is a class that takes on incoming timestamped data, stores
 * them in a minimal way and invokes functions at individual periods.
 */
template<class PacketType>
requires concepts::CompatibleWithSlicer<PacketType>
class StreamSlicer {
public:
	StreamSlicer() = default;

	/**
	 * Add a full packet to the streaming buffer and evaluate jobs.
	 * This function copies the data over.
	 * @param data the packet to be added to the buffer.
	 */
	void accept(const PacketType &data) {
		if constexpr (dv::concepts::EventStorage<PacketType>) {
			mStorePacket.add(data);
		}
		else if constexpr (dv::concepts::DataPacket<PacketType>) {
			mStorePacket.elements.insert(mStorePacket.elements.end(), data.begin(), data.end());
		}
		else {
			mStorePacket.insert(mStorePacket.end(), data.begin(), data.end());
		}
		evaluate();
	}

	/**
	 * Adds a single element of a stream to the slicer buffer and evaluate jobs.
	 * @param element the element to be added to the buffer
	 */
	template<class ElementType>
	requires concepts::Timestamped<ElementType>
	void accept(const ElementType &element) {
		if constexpr (dv::concepts::DataPacket<PacketType>) {
			static_assert(
				std::is_same_v<ElementType,
					std::remove_const_t<dv::concepts::iterable_element_type<decltype(mStorePacket.elements)>>>,
				"Incompatible type for slicer accept method");
			mStorePacket.elements.push_back(element);
		}
		else {
			static_assert(
				std::is_same_v<ElementType, std::remove_const_t<dv::concepts::iterable_element_type<PacketType>>>,
				"Incompatible type for slicer accept method");
			mStorePacket.push_back(element);
		}
		evaluate();
	}

	/**
	 * Adds full stream packet of data to the buffer and evaluates jobs.
	 * @param packet the packet to be added to the buffer
	 */
	void accept(PacketType &&packet) {
		if constexpr (dv::concepts::EventStorage<PacketType>) {
			mStorePacket.add(packet);
		}
		else if constexpr (dv::concepts::DataPacket<PacketType>) {
			mStorePacket.elements.insert(mStorePacket.elements.end(), std::make_move_iterator(packet.elements.begin()),
				std::make_move_iterator(packet.elements.end()));
		}
		else {
			mStorePacket.insert(
				mStorePacket.end(), std::make_move_iterator(packet.begin()), std::make_move_iterator(packet.end()));
		}
		evaluate();
	}

	/**
	 * Adds a number-of-elements triggered job to the Slicer. A job is defined
	 * by its interval and callback function. The slicer calls the callback
	 * function every `n` elements are added to the stream buffer, with the corresponding data.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param n the interval (in number of elements) in which the callback
	 * should be called
	 * @param callback the callback function that gets called on the data
	 * every interval
	 * @return A handle to uniquely identify the job.
	 * @deprecated Use doEveryNumberOfElements() method instead.
	 */
	[[deprecated("Use doEveryNumberOfElements() method instead.")]] int doEveryNumberOfEvents(
		const size_t n, std::function<void(PacketType &)> callback) {
		return doEveryNumberOfElements(n, callback);
	}

	/**
	 * Adds a number-of-elements triggered job to the Slicer. A job is defined
	 * by its interval and callback function. The slicer calls the callback
	 * function every `n` elements are added to the stream buffer, with the corresponding data.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param n the interval (in number of elements) in which the callback
	 * should be called
	 * @param callback the callback function that gets called on the data
	 * every interval
	 * @return A handle to uniquely identify the job.
	 */
	int doEveryNumberOfElements(const size_t n, std::function<void(PacketType &)> callback) {
		mHashCounter += 1;
		mSliceJobs.emplace(std::make_pair(mHashCounter,
			SliceJob(SliceJob::SliceType::NUMBER, 0, n, [callback](const dv::TimeWindow &_, PacketType &packet) {
				callback(packet);
			})));
		return mHashCounter;
	}

	/**
	 * Adds a number-of-elements triggered job to the Slicer. A job is defined
	 * by its interval and callback function. The slicer calls the callback
	 * function every `n` elements are added to the stream buffer, with the corresponding data.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param n the interval (in number of elements) in which the callback should be called
	 * @param callback the callback function that gets called on the data every interval, also passes time window
	 * containing the inter
	 * @return A handle to uniquely identify the job.
	 */
	int doEveryNumberOfElements(const size_t n, std::function<void(const dv::TimeWindow &, PacketType &)> callback) {
		mHashCounter += 1;
		mSliceJobs.emplace(std::make_pair(mHashCounter, SliceJob(SliceJob::SliceType::NUMBER, 0, n, callback)));
		return mHashCounter;
	}

	/**
	 * Adds an element-timestamp-interval triggered job to the Slicer.
	 * A job is defined by its interval and callback function. The slicer
	 * calls the callback whenever the timestamp difference of an incoming
	 * event to the last time the function was called is bigger than the
	 * interval. As the timing is based on event times rather than CPU time,
	 * the actual time periods are not guaranteed, especially with a low event
	 * rate.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param interval the interval in which the callback should be called
	 * @param callback the callback function that gets called on the data
	 * every interval
	 * @return A handle to uniquely identify the job.
	 */
	int doEveryTimeInterval(const dv::Duration interval, std::function<void(const PacketType &)> callback) {
		return doEveryTimeInterval(interval, [callback](const dv::TimeWindow &, const PacketType &data) {
			callback(data);
		});
	}

	/**
	 * Adds an element-timestamp-interval triggered job to the Slicer.
	 * A job is defined by its interval and callback function. The slicer
	 * calls the callback whenever the timestamp difference of an incoming
	 * event to the last time the function was called is bigger than the
	 * interval. As the timing is based on event times rather than CPU time,
	 * the actual time periods are not guaranteed, especially with a low event
	 * rate.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param interval the interval in which the callback should be called
	 * @param callback the callback function that gets called on the data
	 * every interval
	 * @return A handle to uniquely identify the job.
	 * @deprecated Please pass interval parameter using dv::Duration.
	 */
	[[deprecated("Please pass interval parameter using dv::Duration.")]] int doEveryTimeInterval(
		const int64_t microseconds, std::function<void(const PacketType &)> callback) {
		return doEveryTimeInterval(
			dv::Duration(microseconds), [callback](const dv::TimeWindow &, const PacketType &data) {
				callback(data);
			});
	}

	/**
	 * Adds an element-timestamp-interval triggered job to the Slicer.
	 * A job is defined by its interval and callback function. The slicer
	 * calls the callback whenever the timestamp difference of an incoming
	 * event to the last time the function was called is bigger than the
	 * interval. As the timing is based on event times rather than CPU time,
	 * the actual time periods are not guaranteed, especially with a low event
	 * rate.
	 * The (cpu) time interval between individual calls to the function depends
	 * on the physical event rate as well as the bulk sizes of the incoming
	 * data.
	 * @param interval the interval in which the callback should be called
	 * @param callback the callback function that gets called with the time window
	 * information and the data as arguments every interval
	 * @return An id to uniquely identify the job.
	 */
	int doEveryTimeInterval(
		const dv::Duration interval, std::function<void(const dv::TimeWindow &, const PacketType &)> callback) {
		mHashCounter += 1;
		mSliceJobs.emplace(
			std::make_pair(mHashCounter, SliceJob(SliceJob::SliceType::TIME, interval.count(), 0, callback)));
		return mHashCounter;
	}

	/**
	 * Returns true if the slicer contains the slicejob with the provided id
	 * @param jobId the id of the slicejob in question
	 * @return true, if the slicer contains the given slicejob
	 */
	[[nodiscard]] bool hasJob(const int jobId) const {
		return mSliceJobs.contains(jobId);
	}

	/**
	 * Removes the given job from the list of current jobs.
	 * @param jobId The job id to be removed
	 */
	void removeJob(const int jobId) {
		if (!hasJob(jobId)) {
			return;
		}
		mSliceJobs.erase(jobId);
	}

	/**
	 * Modifies the time interval of the supplied job to the requested value
	 * @param jobId  the job whose time interval should be changed
	 * @param timeInterval the new time interval value
	 * @deprecated Please pass time interval as dv::Duration instead.
	 */
	[[deprecated("Please pass time interval as dv::Duration instead.")]] void modifyTimeInterval(
		const int jobId, const int64_t timeInterval) {
		modifyTimeInterval(jobId, dv::Duration(timeInterval));
	}

	/**
	 * Modifies the time interval of the supplied job to the requested value
	 * @param jobId  the job whose time interval should be changed
	 * @param timeInterval the new time interval value
	 */
	void modifyTimeInterval(const int jobId, const dv::Duration timeInterval) {
		if (!hasJob(jobId)) {
			return;
		}
		mSliceJobs[jobId].setTimeInterval(timeInterval.count());
	}

	/**
	 * Modifies the number interval of the supplied job to the requested value
	 * @param jobId the job whose number interval should be changed
	 * @param numberInterval the new number interval value
	 */
	void modifyNumberInterval(const int jobId, const size_t numberInterval) {
		if (!hasJob(jobId)) {
			return;
		}
		mSliceJobs[jobId].setNumberInterval(numberInterval);
	}

private:
	/**
	 * __INTERNAL USE ONLY__
	 * A single job of the EventStreamSlicer
	 */
	class SliceJob {
	public:
		enum class SliceType {
			NUMBER,
			TIME
		};

		/**
		 * __INTERNAL USE ONLY__
		 * Creates a new SliceJob of a certain type, interval and callback
		 * @param type The type of periodicity. Can be either NUMBER or TIME
		 * @param timeInterval The interval at which
		 * the job should be executed
		 * @param numberInterval The interval at which
		 * the job should be executed
		 * @param callback The callback function to call on execution.
		 */
		SliceJob(const SliceType type, const int64_t timeInterval, const size_t numberInterval,
			std::function<void(const dv::TimeWindow &, PacketType &)> callback) :
			mType(type),
			mCallback(std::move(callback)),
			mTimeInterval(timeInterval),
			mNumberInterval(numberInterval),
			mLastCallEndTime(0),
			mLastCallEnd(0) {
		}

		SliceJob() = default;

		/**
		 * __INTERNAL USE ONLY__
		 * This function establishes how much fresh data is availble
		 * and how often the callback can be executed on this fresh data.
		 * it then creates slices of the data and executes the callback as
		 * often as possible.
		 * @param packet the storage packet to slice on.
		 */
		void run(const PacketType &packet) {
			if (dv::packets::getPacketSize(packet) == 0) {
				return;
			}

			if (mType == SliceType::NUMBER) {
				while (dv::packets::getPacketSize(packet) - mLastCallEnd >= mNumberInterval) {
					PacketType slice;
					if constexpr (dv::concepts::DataPacket<PacketType>) {
						slice.elements = sliceByNumber(packet.elements, mLastCallEnd, mNumberInterval);
					}
					else {
						slice = sliceByNumber(packet, mLastCallEnd, mNumberInterval);
					}
					mLastCallEnd = mLastCallEnd + mNumberInterval;
					mCallback(dv::packets::getPacketTimeWindow(slice), slice);
				}
			}

			if (mType == SliceType::TIME) {
				if (mLastCallEndTime == 0) { // initialize with the lowest time
					mLastCallEndTime = dv::packets::getPacketTimestamp<dv::packets::Timestamp::START>(packet);
				}

				while (dv::packets::getPacketTimestamp<dv::packets::Timestamp::END>(packet) - mLastCallEndTime
					   >= mTimeInterval) {
					PacketType slice;
					if constexpr (dv::concepts::DataPacket<PacketType>) {
						slice.elements = sliceByTime(
							packet.elements, mLastCallEndTime, mLastCallEndTime + mTimeInterval, mLastCallEnd);
					}
					else {
						slice = sliceByTime(packet, mLastCallEndTime, mLastCallEndTime + mTimeInterval, mLastCallEnd);
					}
					const dv::TimeWindow window(mLastCallEndTime, mLastCallEndTime + mTimeInterval);
					mLastCallEndTime = mLastCallEndTime + mTimeInterval;
					mCallback(window, slice);
				}
			}
		}

		/**
		 * __INTERNAL USE ONLY__
		 * Sets the time interval to the supplied value
		 * @param timeInterval the new time interval to use
		 */
		void setTimeInterval(const int64_t timeInterval) {
			if (mType != SliceType::TIME) {
				throw std::invalid_argument("Setting a new number interval to a time based slicing job");
			}
			mTimeInterval = timeInterval;
		}

		/**
		 * __INTERNAL USE ONLY__
		 * Sets the number interval to the supplied value
		 * @param numberInterval the new interval to use
		 */
		void setNumberInterval(const size_t numberInterval) {
			if (mType != SliceType::NUMBER) {
				throw std::invalid_argument("Setting a new time interval to a number based slicing job");
			}
			mNumberInterval = numberInterval;
		}

		size_t mLastCallEnd = 0;

	private:
		template<class ElementVector>
		[[nodiscard]] static inline ElementVector sliceByNumber(
			const ElementVector &packet, const size_t fromIndex, const size_t number) {
			if constexpr (dv::concepts::EventStorage<ElementVector>) {
				return packet.slice(fromIndex, number);
			}
			else {
				if (fromIndex + number > dv::packets::getPacketSize(packet)) {
					throw std::out_of_range("Trying to slice data outside of range");
				}
				return ElementVector(
					std::next(packet.begin(), fromIndex), std::next(packet.begin(), fromIndex + number));
			}
		}

		template<class ElementVector>
		[[nodiscard]] static inline ElementVector sliceByTime(
			const ElementVector &packet, const int64_t start, const int64_t end, size_t &endIndex) {
			if constexpr (dv::concepts::EventStorage<ElementVector>) {
				size_t startIndex;
				return packet.sliceTime(start, end, startIndex, endIndex);
			}
			else {
				const auto comparator = [](const auto &elem, const auto &time) {
					return dv::packets::getTimestamp(elem) < time;
				};

				auto lowerBound = std::lower_bound(packet.begin(), packet.end(), start, comparator);

				if (lowerBound == packet.end()) {
					endIndex = 0;
					return ElementVector();
				}

				// Upper is not going to be before lower, since time is guaranteed to be increasing. We can use
				// lowerBound as starting point
				auto upperBound = std::lower_bound(lowerBound, packet.end(), end, comparator);

				endIndex = std::distance(packet.begin(), upperBound);
				return ElementVector(lowerBound, upperBound);
			}
		}

		SliceType mType = SliceType::TIME;
		const std::function<void(const TimeWindow &, PacketType &)> mCallback;
		int64_t mTimeInterval    = 0;
		size_t mNumberInterval   = 0;
		int64_t mLastCallEndTime = 0;
	};

	/// Global storage packet that holds just as many data elements as minimally required for all outstanding calls
	PacketType mStorePacket;

	/// List of all the sliceJobs
	std::map<int, SliceJob> mSliceJobs;
	int mHashCounter = 0;

	/**
	 * Should get called as soon as there is fresh data available.
	 * It loops through all jobs and determines if they can run on the new data.
	 * The jobs get executed as often as possible. Afterwards, all data that has
	 * been processed by all jobs gets discarded.
	 */
	void evaluate() {
		// run jobs
		for (auto &jobTuple : mSliceJobs) {
			jobTuple.second.run(mStorePacket);
		}

		// find  border of the end of last call
		size_t lowerBound;
		if constexpr (dv::concepts::DataPacket<PacketType>) {
			lowerBound = mStorePacket.elements.size();
		}
		else {
			lowerBound = mStorePacket.size();
		}

		for (auto &jobTuple : mSliceJobs) {
			lowerBound = std::min(lowerBound, jobTuple.second.mLastCallEnd);
		}

		// discard fully processed events and readjust call boundaries of jobs
		if constexpr (dv::concepts::EventStorage<PacketType>) {
			mStorePacket = mStorePacket.slice(lowerBound);
		}
		else if constexpr (dv::concepts::DataPacket<PacketType>) {
			mStorePacket.elements.erase(
				mStorePacket.elements.begin(), std::next(mStorePacket.elements.begin(), lowerBound));
		}
		else {
			mStorePacket.erase(mStorePacket.begin(), std::next(mStorePacket.begin(), lowerBound));
		}

		for (auto &jobTuple : mSliceJobs) {
			jobTuple.second.mLastCallEnd = jobTuple.second.mLastCallEnd - lowerBound;
		}
	}
};

} // namespace dv

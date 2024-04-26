#pragma once

#include "../core/frame.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"

#include <valarray>

namespace dv {

/**
 * A base class for noise filter implementations. Handles data input and output, derived classes only have to
 * implement a retain function that tests whether event should be retained or discarded.
 */
template<class EventStoreClass = dv::EventStore>
class EventFilterBase {
protected:
	EventStoreClass buffer;

	int64_t highestProcessedTime = -1;

	size_t numIncomingEvents = 0;

	size_t numOutgoingEvents = 0;

public:
	/**
	 * Accepts incoming events.
	 * @param store 	Event packet.
	 */
	void accept(const EventStoreClass &store) {
		if (store.isEmpty()) {
			return;
		}

		if (store.getLowestTime() < highestProcessedTime) {
			throw std::out_of_range{"Tried adding event store to store out of order. Ignoring packet."};
		}

		buffer.add(store);
	}

	/**
	 * A function to be implemented by derived class which tests whether given event
	 * should be retained or discarded.
	 * @param event 	An event to be checked.
	 * @return 			Return true if the event is to be retained or false to discard the event.
	 */
	virtual inline bool retain(const typename EventStoreClass::value_type &event) noexcept = 0;

	/**
	 * Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.
	 * @return
	 */
	[[nodiscard]] EventStoreClass generateEvents() {
		if (!buffer.isEmpty()) {
			numIncomingEvents += buffer.size();

			std::shared_ptr<typename EventStoreClass::packet_type> packet
				= std::make_shared<typename EventStoreClass::packet_type>();
			packet->elements.reserve(buffer.size());

			for (const auto &event : buffer) {
				if (retain(event)) {
					packet->elements.push_back(event);
				}
			}

			packet->elements.shrink_to_fit();
			numOutgoingEvents    += packet->elements.size();
			highestProcessedTime = buffer.getHighestTime();

			buffer = EventStoreClass{};
			return EventStoreClass(std::const_pointer_cast<typename EventStoreClass::const_packet_type>(packet));
		}

		return {};
	}

	/**
	 * Get number of total events that were accepted by the noise filter.
	 * @return 		Total number of incoming events to this filter instance.
	 */
	[[nodiscard]] size_t getNumIncomingEvents() const {
		return numIncomingEvents;
	}

	/**
	 * Total number of outgoing events from this filter instance.
	 * @return 		Total number of outgoing events from this filter instance.
	 */
	[[nodiscard]] size_t getNumOutgoingEvents() const {
		return numOutgoingEvents;
	}

	/**
	 * Get the reduction factor of this filter. It's a fraction representation of events that were discard by this
	 * filter compared to the amount of incoming events.
	 * @return 		Reduction factor value.
	 */
	[[nodiscard]] float getReductionFactor() const {
		if (numIncomingEvents > 0) {
			return 1.f - (static_cast<float>(numOutgoingEvents) / static_cast<float>(numIncomingEvents));
		}
		else {
			return 0.f;
		}
	}

	virtual ~EventFilterBase() = default;

	/**
	 * Retrieve filtered events using output stream operator.
	 * @param out 		Filtered events.
	 * @return
	 */
	EventStoreClass &operator>>(EventStoreClass &out) {
		out = generateEvents();
		return out;
	}
};

/**
 * Event filter that filters events based on a given ROI.
 * @tparam EventStoreClass 	Type of event store
 */
template<class EventStoreClass = dv::EventStore>
class EventRegionFilter : public EventFilterBase<EventStoreClass> {
protected:
	cv::Rect roi;

public:
	/**
	 * Filter event based on an ROI.
	 * @param roi		Region of interest, events outside of this region will be discarded.
	 */
	explicit EventRegionFilter(const cv::Rect &roi) : roi(roi) {
	}

	/**
	 * Test whether event belongs to an ROI.
	 * @param event 	Event to be checked.
	 * @return 			True if event belongs to ROI, false otherwise.
	 */
	[[nodiscard]] inline bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		return roi.contains(cv::Point2i(event.x(), event.y()));
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline EventRegionFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}
};

static_assert(concepts::EventFilter<EventRegionFilter<>, dv::EventStore>);

/**
 * Event filter based on polarity.
 * @tparam EventStoreClass 	Type of event store
 */
template<class EventStoreClass = dv::EventStore>
class EventPolarityFilter : public EventFilterBase<EventStoreClass> {
protected:
	bool polarity;

public:
	/**
	 * Construct an event filter which filters out only events of given polarity.
	 * @param polarity 		Extract events only of matching polarity.
	 */
	explicit EventPolarityFilter(const bool polarity) : polarity(polarity) {
	}

	/**
	 * Test whether event is of configured polarity.
	 * @param event 	Event to be checked.
	 * @return 			True if event has the expected polarity, false otherwise.
	 */
	[[nodiscard]] inline bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		return event.polarity() == polarity;
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline EventPolarityFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}
};

static_assert(concepts::EventFilter<EventPolarityFilter<>, dv::EventStore>);

/**
 * Event filter based on multiple event filter applied sequentially. Internally stores
 * any added filters and
 * @tparam EventStoreClass 	Type of event store
 */
template<class EventStoreClass = dv::EventStore>
class EventFilterChain : public EventFilterBase<EventStoreClass> {
protected:
	std::vector<std::shared_ptr<dv::EventFilterBase<EventStoreClass>>> filters;

public:
	/**
	 * Add a filter to the chain of filtering.
	 * @param filter
	 */
	void addFilter(std::shared_ptr<dv::EventFilterBase<EventStoreClass>> filter) {
		filters.push_back(std::move(filter));
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline EventFilterChain &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}

	/**
	 * Test whether event is of configured polarity.
	 * @param event 	Event to be checked.
	 * @return 			True if event has the expected polarity, false otherwise.
	 */
	[[nodiscard]] inline bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		for (const auto &filter : filters) {
			if (!filter->retain(event)) {
				return false;
			}
		}

		return true;
	}
};

template<class EventStoreClass = dv::EventStore>
class RefractoryPeriodFilter : public EventFilterBase<EventStoreClass> {
private:
	dv::TimeSurface mTimeSurface;

	int64_t mRefractoryPeriod;

public:
	/**
	 * Refractory period filter discards any events that are registered at a pixel location that already
	 * had an event within the refractory period. Refractory period should be relatively small
	 * value (in the range of one or a few hundred microseconds).
	 * @param resolution 			Sensor resolution.
	 * @param refractoryPeriod 		Refractory period duration.
	 */
	explicit RefractoryPeriodFilter(
		const cv::Size &resolution, const dv::Duration refractoryPeriod = dv::Duration(250)) :
		mTimeSurface(resolution),
		mRefractoryPeriod(refractoryPeriod.count()) {
	}

	/**
	 * Test whether event satisfies (is larger than) refractory period test.
	 * @param event 	Event to be tested.
	 * @return 			True - there were no events within the refractory period at that pixel location,
	 * 					false otherwise.
	 */
	inline bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		int64_t &tsTime = mTimeSurface(event.y(), event.x());
		bool isSignal   = (event.timestamp() - tsTime) >= mRefractoryPeriod;
		tsTime          = event.timestamp();

		return isSignal;
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline RefractoryPeriodFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}

	/**
	 * Get the refractory period.
	 * @return 		Currently configured refractory period.
	 */
	[[nodiscard]] dv::Duration getRefractoryPeriod() const {
		return dv::Duration(mRefractoryPeriod);
	}

	/**
	 * Set a new refractory period value.
	 * @param refractoryPeriod 		New refractory period value.
	 */
	void setRefractoryPeriod(const dv::Duration refractoryPeriod) {
		mRefractoryPeriod = refractoryPeriod.count();
	}
};

static_assert(dv::concepts::EventFilter<RefractoryPeriodFilter<>, dv::EventStore>);

template<class EventStoreClass = dv::EventStore>
class EventMaskFilter : public EventFilterBase<EventStoreClass> {
private:
	cv::Mat mMask;

public:
	/**
	 * Create an event masking filter. Discards any events that happen on coordinates where mask has a zero
	 * value and retains all events with coordinates where mask has a non-zero value.
	 * @param mask 		The mask to be applied (requires CV_8UC1 type).
	 * @throws InvalidArgument	Exception thrown if the mask is of incorrect type.
	 */
	explicit EventMaskFilter(const cv::Mat &mask) : mMask(mask) {
		if (mask.type() != CV_8UC1) {
			throw dv::exceptions::InvalidArgument<int>(
				"Invalid mask type, expecting a mask of a single channel 8-bit unsigned"
				"integer matrix (CV_8UC1).",
				mask.type());
		}
	}

	bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		return mMask.at<uint8_t>(event.y(), event.x()) > 0;
	}

	/**
	 * Get the mask that is currently applied.
	 * @return
	 */
	[[nodiscard]] const cv::Mat &getMask() const {
		return mMask;
	}

	/**
	 * Set a new mask to this filter.
	 * @param mask 		The mask to be applied (requires CV_8UC1 type).
	 */
	void setMask(const cv::Mat &mask) {
		mMask = mask;
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline EventMaskFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}
};

static_assert(dv::concepts::EventFilter<EventMaskFilter<>, dv::EventStore>);

} // namespace dv

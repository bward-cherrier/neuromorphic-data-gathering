#pragma once

#include "core.hpp"
#include "filters.hpp"

namespace dv {

/**
 * Function that creates perfect hash for 2d coordinates.
 * @param x x coordinate
 * @param y y coordinate
 * @return a 64 bit hash that uniquely identifies the coordinates
 */
inline uint32_t coordinateHash(const int16_t x, const int16_t y) {
	return (static_cast<uint32_t>(x) << 16u) | static_cast<uint32_t>(y);
}

/**
 * Extracts only the events that are within the defined region of interest.
 * This function copies the events from the in EventStore into the given
 * out EventStore, if they intersect with the given region of interest rectangle.
 * @param in The EventStore to operate on. Won't be modified.
 * @param out The EventStore to put the ROI events into. Will get modified.
 * @param roi The rectangle with the region of interest.
 */
template<class EventStoreType>
inline void roiFilter(const EventStoreType &in, EventStoreType &out, const cv::Rect &roi) {
	// in-place filtering is not supported
	dv::runtime_assert(&in != &out, "in-place filtering is not supported");

	dv::EventRegionFilter<EventStoreType> filter(roi);

	filter.accept(in);
	out = filter.generateEvents();
}

/**
 * Filters events by polarity. Only events that exhibit the same polarity as given in
 * polarity are kept.
 * @param in Incoming EventStore to operate on. Won't get modified.
 * @param out The outgoing EventStore to store the kept events on
 * @param polarity The polarity of the events that should be kept
 */
template<class EventStoreType>
inline void polarityFilter(const EventStoreType &in, EventStoreType &out, bool polarity) {
	// in-place filtering is not supported
	dv::runtime_assert(&in != &out, "in-place filtering is not supported");

	dv::EventPolarityFilter<EventStoreType> filter(polarity);

	filter.accept(in);
	out = filter.generateEvents();
}

/**
 * Filter event with a coordinate mask. Discards any events that happen on coordinates where mask has a zero
 * value and retains all events with coordinates where mask has a non-zero value.
 * @tparam EventStoreType Class for the event store container.
 * @param in Incoming EventStore to operate on. Won't get modified.
 * @param out The outgoing EventStore to store the kept events on
 * @param mask The mask to be applied (requires CV_8UC1 type).
 */
template<class EventStoreType>
inline void maskFilter(const EventStoreType &in, EventStoreType &out, const cv::Mat &mask) {
	// in-place filtering is not supported
	dv::runtime_assert(&in != &out, "in-place filtering is not supported");

	dv::EventMaskFilter<EventStoreType> filter(mask);

	filter.accept(in);
	out = filter.generateEvents();
}

/**
 * Projects the event coordinates onto a smaller range. The x- and y-coordinates the divided
 * by xFactor and yFactor respectively and floored to the next integer. This forms the new
 * coordinates of the event. Due to the nature of this, it can happen that multiple events
 * end up happening simultaneously at the same location. This is still a valid event stream,
 * as time keeps monotonically increasing, but is something that is unlikely to be generated
 * by an event camera.
 * @param in The EventStore to operate on. Won't get modified
 * @param out The outgoing EventStore to store the projected events on
 * @param xDivision Division factor for the x-coordinate for the events
 * @param yDivision Division factor for the y-coordinate of the events
 */
template<class EventStoreType>
inline void scale(const EventStoreType &in, EventStoreType &out, double xDivision, double yDivision) {
	// in-place filtering is not supported
	dv::runtime_assert(&in != &out, "in-place filtering is not supported");

	for (const auto &event : in) {
		out.emplace_back(event.timestamp(), static_cast<int16_t>(event.x() / xDivision),
			static_cast<int16_t>(event.y() / yDivision), event.polarity());
	}
}

/**
 * Computes and returns a rectangle with dimensions such that all the events
 * in the given `EventStore` fall into the bounding box.
 * @param packet The EventStore to work on
 * @return The smallest possible rectangle that contains all the events in packet.
 */
template<class EventStoreType>
inline cv::Rect boundingRect(const EventStoreType &packet) {
	if (packet.isEmpty()) {
		return {0, 0, 0, 0};
	}

	int16_t minX = std::numeric_limits<int16_t>::max();
	int16_t maxX = 0;
	int16_t minY = std::numeric_limits<int16_t>::max();
	int16_t maxY = 0;

	for (const auto &event : packet) {
		minX = std::min(event.x(), minX);
		maxX = std::max(event.x(), maxX);
		minY = std::min(event.y(), minY);
		maxY = std::max(event.y(), maxY);
	}

	// top left coordinates and width / height, you need to add +1 to get correct size
	return {minX, minY, (maxX - minX) + 1, (maxY - minY) + 1};
}

} // namespace dv

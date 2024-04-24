#pragma once

#include "../data/event_base.hpp"

namespace dv {
/**
 * The EventColor enum contains the color of the Bayer color filter for a specific event address.
 * WHITE means White/No Filter.
 * Please take into account that there are usually twice as many green pixels as there are red or blue ones.
 */
enum EventColor : int8_t {
	WHITE = 0,
	RED   = 1,
	GREEN = 2,
	BLUE  = 3,
};

/**
 * Color pixel block arrangement on the sensor. The sensor usually contain one red, one blue, and two green pixels.
 * They can be arranged in different order, so exact color extraction, the pixel arrangement needs to be known.
 */
enum PixelArrangement : int8_t {
	RGBG = 0,
	GRGB = 1,
	GBGR = 2,
	BGRG = 3
};

/**
 * Address to Color mapping for events based on Bayer filter.
 */
static constexpr EventColor colorKeys[4][4] = {
	{EventColor::RED,   EventColor::GREEN, EventColor::GREEN, EventColor::BLUE },
	{EventColor::GREEN, EventColor::BLUE,  EventColor::RED,   EventColor::GREEN},
	{EventColor::GREEN, EventColor::RED,   EventColor::BLUE,  EventColor::GREEN},
	{EventColor::BLUE,  EventColor::GREEN, EventColor::GREEN, EventColor::RED  },
};

/**
 * Determine the color of the Bayer color filter for a specific event, based on its address.
 * Please take into account that there are usually twice as many green pixels as there are red or blue ones.
 *
 * @param evt event to determine filter color for.
 * @param pixelArrangement color pixel arrangement for a sensor.
 *
 * @return filter color.
 */
[[nodiscard]] inline EventColor colorForEvent(
	const Event &evt, const PixelArrangement arrangement = PixelArrangement::RGBG) {
	const auto x   = static_cast<uint32_t>(evt.x());
	const auto y   = static_cast<uint32_t>(evt.y());
	const auto idx = static_cast<size_t>(((x & 0x01) << 1) | (y & 0x01));

	return colorKeys[arrangement][idx];
}

} // namespace dv

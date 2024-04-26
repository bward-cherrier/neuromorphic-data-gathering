#pragma once

#include <cstdint>

namespace dv::measurements {

/**
 * A depth measurement structure that contains a timestamped measurement of depth.
 */
struct Depth {
	/**
	 * UNIX Microsecond timestamp
	 */
	int64_t mTimestamp;

	/**
	 * Depth measurement value, expected to be in meters.
	 */
	float mDepth;

	Depth(int64_t timestamp, float depth) : mTimestamp(timestamp), mDepth(depth) {
	}
};

} // namespace dv::measurements

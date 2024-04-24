#pragma once

#include <chrono>

namespace dv {

using TimestampClock      = std::chrono::system_clock;
using TimestampResolution = std::chrono::microseconds;

/**
 * Duration type that stores microsecond time period.
 */
using Duration = TimestampResolution;

/**
 * Timepoint type that stores microsecond time point related to system clock.
 */
using TimePoint = std::chrono::time_point<TimestampClock, TimestampResolution>;

/**
 * Convert a 64-bit integer microsecond timestamp into a chrono time-point.
 * @param timestamp		64-bit integer microsecond timestamp
 * @return 				Chrono time point (microseconds, system clock).
 */
[[nodiscard]] inline TimePoint toTimePoint(const int64_t timestamp) {
	return TimePoint{TimestampResolution{timestamp}};
}

/**
 * Convert a chrono time-point into a 64-bit integer microsecond timestamp.
 * @param timestamp		Chrono time point (microseconds, system clock).
 * @return 				64-bit integer microsecond timestamp
 */
[[nodiscard]] inline int64_t fromTimePoint(const TimePoint timepoint) {
	return static_cast<int64_t>(timepoint.time_since_epoch().count());
}

/**
 * @return Current system clock timestamp in microseconds as 64-bit integer.
 */
[[nodiscard]] inline int64_t now() {
	return fromTimePoint(std::chrono::time_point_cast<Duration>(TimestampClock::now()));
}

} // namespace dv

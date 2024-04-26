#pragma once

#include "time.hpp"

namespace dv {

struct TimeWindow {
	int64_t startTime;
	int64_t endTime;

	TimeWindow(const int64_t timestamp, const dv::Duration duration) :
		startTime(timestamp),
		endTime(startTime + duration.count()) {
	}

	TimeWindow(const int64_t startTime, const int64_t endTime) : startTime(startTime), endTime(endTime) {
	}

	[[nodiscard]] dv::Duration duration() const {
		return dv::Duration(endTime - startTime);
	}
};

} // namespace dv

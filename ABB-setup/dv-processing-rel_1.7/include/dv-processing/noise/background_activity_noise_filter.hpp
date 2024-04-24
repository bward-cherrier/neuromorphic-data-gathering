#pragma once

#include "../core/filters.hpp"

namespace dv::noise {

template<class EventStoreClass = dv::EventStore>
class BackgroundActivityNoiseFilter : public EventFilterBase<EventStoreClass> {
protected:
	cv::Size mResolutionLimits;

	dv::TimeSurface mTimeSurface;

	int64_t mBackgroundActivityDuration = 2000;

	inline bool doBackgroundActivityLookup_unsafe(int16_t x, int16_t y, int64_t timestamp) {
		y--;
		if ((timestamp - mTimeSurface(y, x)) < mBackgroundActivityDuration) {
			return true;
		}

		if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
			return true;
		}

		if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
			return true;
		}
		y++;

		if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
			return true;
		}

		if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
			return true;
		}

		y++;
		if ((timestamp - mTimeSurface(y, x)) < mBackgroundActivityDuration) {
			return true;
		}

		if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
			return true;
		}

		if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
			return true;
		}
		return false;
	}

	inline bool doBackgroundActivityLookup(int16_t x, int16_t y, int64_t timestamp) {
		// Compute map limits.
		bool notBorderLeft  = (x != 0);
		bool notBorderDown  = (y != mResolutionLimits.height);
		bool notBorderRight = (x != mResolutionLimits.width);
		bool notBorderUp    = (y != 0);

		if (notBorderLeft && notBorderDown && notBorderRight && notBorderUp) {
			return doBackgroundActivityLookup_unsafe(x, y, timestamp);
		}

		// Background Activity filter: if difference between current timestamp
		// and stored neighbor timestamp is smaller than given time limit, it
		// means the event is supported by a neighbor and thus valid. If it is
		// bigger, then the event is not supported, and we need to check the
		// next neighbor. If all are bigger, the event is invalid.
		if (notBorderLeft) {
			if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
				return true;
			}
		}

		if (notBorderRight) {
			if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
				return true;
			}
		}

		if (notBorderUp) {
			y--;
			if ((timestamp - mTimeSurface(y, x)) < mBackgroundActivityDuration) {
				return true;
			}

			if (notBorderLeft) {
				if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
					return true;
				}
			}

			if (notBorderRight) {
				if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
					return true;
				}
			}
			y++;
		}

		if (notBorderDown) {
			y++;
			if ((timestamp - mTimeSurface(y, x)) < mBackgroundActivityDuration) {
				return true;
			}

			if (notBorderLeft) {
				if ((timestamp - mTimeSurface(y, x - 1)) < mBackgroundActivityDuration) {
					return true;
				}
			}

			if (notBorderRight) {
				if ((timestamp - mTimeSurface(y, x + 1)) < mBackgroundActivityDuration) {
					return true;
				}
			}
		}

		return false;
	}

public:
	/**
	 * Initiate a background activity noise filter, which test the neighbourhoods of incoming events for
	 * other supporting events that happened within the background activity period.
	 * @param resolution 					Sensor resolution.
	 * @param backgroundActivityDuration 	Background activity duration.
	 */
	explicit BackgroundActivityNoiseFilter(
		const cv::Size &resolution, const dv::Duration backgroundActivityDuration = dv::Duration(2000)) :
		mResolutionLimits(resolution.width - 1, resolution.height - 1),
		mTimeSurface(resolution),
		mBackgroundActivityDuration(backgroundActivityDuration.count()) {
	}

	/**
	 * Test the background activity, if the event neighbourhood has at least one event that was triggered within
	 * the background activity duration, the event will not be considered noise and should be retained, and discarded
	 * otherwise.
	 * @param evt 		Event to be checked.
	 * @return 			True to retain event, false to discard.
	 */
	inline bool retain(const typename EventStoreClass::value_type &evt) noexcept override {
		bool isSignal = doBackgroundActivityLookup(evt.x(), evt.y(), evt.timestamp());

		mTimeSurface(evt.y(), evt.x()) = evt.timestamp();

		return isSignal;
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline BackgroundActivityNoiseFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}

	/**
	 * Get currently configured background activity duration value.
	 * @return 		Background activity duration value.
	 */
	[[nodiscard]] dv::Duration getBackgroundActivityDuration() const {
		return dv::Duration(mBackgroundActivityDuration);
	}

	/**
	 * Set new background activity duration value.
	 * @param backgroundActivityDuration 	Background activity duration value.
	 */
	void setBackgroundActivityDuration(const dv::Duration backgroundActivityDuration) {
		BackgroundActivityNoiseFilter::mBackgroundActivityDuration = backgroundActivityDuration.count();
	}
};

static_assert(dv::concepts::EventFilter<BackgroundActivityNoiseFilter<>, dv::EventStore>);

} // namespace dv::noise

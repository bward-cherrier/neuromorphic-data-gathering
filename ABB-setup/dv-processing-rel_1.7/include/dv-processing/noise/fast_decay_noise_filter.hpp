#pragma once

#include "../core/filters.hpp"

namespace dv::noise {

template<class EventStoreClass = dv::EventStore>
class FastDecayNoiseFilter : public EventFilterBase<EventStoreClass> {
private:
	int mSubdivisionFactor = 4;

	cv::Mat mDecayLUT;

	dv::TimeSurface mTimeSurface;

	float mNoiseThreshold = 6.f;

	float mHalfLifeMicros = 10'000.f; // 10ms

public:
	/**
	 * Create a fast decay noise filter. This filter uses a concept that performs a fast decay on a low
	 * resolution representation of the image and checks whether corresponding neighbourhood of the event
	 * has recent activity.
	 * @param resolution 			Sensor resolution.
	 * @param halfLife				Half-life is the amount of time it takes for the internal event counter to halve.
	 * 								Decreasing this will increase the strength of the noise filter (cause it to reject
	 * 								more events).
	 * @param subdivisionFactor 	Subdivision factor, this is used calculate a low resolution image dimensions
	 * 								used for the fast decay operations.
	 * @param noiseThreshold 		Noise threshold value, amount of filtered events can be increased by decreasing
	 * 								this value.
	 */
	explicit FastDecayNoiseFilter(const cv::Size &resolution, const dv::Duration halfLife = dv::Duration(10'000),
		const int subdivisionFactor = 4, const float noiseThreshold = 6.f) :
		mSubdivisionFactor(subdivisionFactor),
		mDecayLUT(static_cast<int>(resolution.height / subdivisionFactor) + 1,
			static_cast<int>(resolution.width / subdivisionFactor) + 1, CV_32FC1, cv::Scalar(0.)),
		mTimeSurface(static_cast<uint32_t>(resolution.height / subdivisionFactor) + 1,
			static_cast<uint32_t>(resolution.width / subdivisionFactor) + 1),
		mNoiseThreshold(noiseThreshold),
		mHalfLifeMicros(static_cast<float>(halfLife.count())) {
	}

	/**
	 * Test whether to retain this event.
	 * @param event 	Event to be checked.
	 * @return 			True to retain an event, false to discard it.
	 */
	inline bool retain(const typename EventStoreClass::value_type &event) noexcept override {
		auto x = static_cast<int16_t>(event.x() / mSubdivisionFactor);
		auto y = static_cast<int16_t>(event.y() / mSubdivisionFactor);

		int64_t &lastFiredTimestamp = mTimeSurface(y, x);

		// This could be just a linear coefficient to save expensive exponential computations.
		const float decayMult
			= std::exp2(-static_cast<float>(event.timestamp() - lastFiredTimestamp) / mHalfLifeMicros);

		lastFiredTimestamp = event.timestamp();

		auto &value = mDecayLUT.at<float>(y, x);

		value = (value * decayMult) + 1.f;
		return value > mNoiseThreshold;
	}

	/**
	 * Accept events using the input stream operator.
	 * @param events 	Input events.
	 * @return
	 */
	inline FastDecayNoiseFilter &operator<<(const EventStoreClass &events) {
		accept(events);
		return *this;
	}

	/**
	 * Get the currently configured noise threshold.
	 * @return 		Noise threshold value.
	 */
	[[nodiscard]] float getNoiseThreshold() const {
		return mNoiseThreshold;
	}

	/**
	 * Set a new noise threshold value.
	 * @param noiseThreshold 	Noise threshold value.
	 */
	void setNoiseThreshold(const float noiseThreshold) {
		mNoiseThreshold = noiseThreshold;
	}

	/**
	 * Get the current configured half-life value.
	 *
	 * Half-life is the amount of time it takes for the internal event counter to halve. Decreasing this will increase
	 * the strength of the noise filter (cause it to reject more events).
	 * @return			Currently configured event counter half life value.
	 */
	[[nodiscard]] dv::Duration getHalfLife() const {
		return dv::Duration(static_cast<int64_t>(mHalfLifeMicros));
	}

	/**
	 * Set a new counter half-life value.
	 *
	 * Half-life is the amount of time it takes for the internal event counter to halve. Decreasing this will increase
	 * the strength of the noise filter (cause it to reject more events).
	 * @param halfLife 	New event counter half life value.
	 */
	void setHalfLife(const dv::Duration halfLife) {
		mHalfLifeMicros = static_cast<float>(halfLife.count());
	}
};

static_assert(dv::concepts::EventFilter<FastDecayNoiseFilter<>, dv::EventStore>);

} // namespace dv::noise

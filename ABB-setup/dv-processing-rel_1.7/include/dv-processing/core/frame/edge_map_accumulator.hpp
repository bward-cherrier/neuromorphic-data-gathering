#pragma once

#include "accumulator_base.hpp"

namespace dv {
/**
 * `dv::EdgeMapAccumulator` accumulates events in a histogram representation
 * with configurable contribution, but it is more efficient compared to generic
 * accumulator since it uses 8-bit unsigned integers as internal memory type.
 *
 * The EdgeMapAccumulator behaves the same as a generic `dv::Accumulator` with STEP
 * decay function, neutral and minimum value of 0.0, maximum value of 1.0 and
 * configurable event contribution. The difference is that it doesn't use floating
 * point numbers for the potential surface representation. The output data type of
 * this accumulator is single channel 8-bit unsigned integer (CV_8UC1). Accumulation
 * is performed using integer operations as well.
 * Due to performance, no check on the event coordinates inside image plane is performed,
 * unless compiled specifically in DEBUG mode.
 * Events out of the image plane bounds will result in undefined behaviour, or program
 * termination in DEBUG mode.
 */
class EdgeMapAccumulator : public AccumulatorBase {
protected:
	/**
	 * Buffer to keep the latest events
	 */
	dv::EventStore buffer;

	/**
	 * Max unsigned byte value
	 */
	uint8_t maxByteValue = 255;

	/**
	 * Default contribution
	 */
	float contribution = 0.25f;
	/**
	 * Increment value for a single event
	 */
	uint8_t drawIncrement = (static_cast<uint8_t>(static_cast<float>(maxByteValue) * contribution));

	/**
	 * A look-up table for increment values at each possible pixel value.
	 */
	std::vector<uint8_t> incrementLUT;

	bool ignorePolarity = true;

	float neutralValue = 0.f;

	uint8_t neutralByteValue = 0;

	float decay = 1.0;

	std::vector<uint8_t> decayLUT;

	cv::Mat imageBuffer;

	enum class DecayMode {
		None,
		Full,
		Decay
	};

	DecayMode decayMode = DecayMode::Full;

public:
	/// Decay coefficient value to disable any decay - zero decay
	static constexpr float DECAY_NONE = 0.0f;

	/// Maximum decay coefficient value which causes reset of pixels into neutral potential at each frame generation
	static constexpr float DECAY_FULL = 1.0f;

	/**
	 * Create a pixel accumulator with known image dimensions and event contribution.
	 * @param resolution 		Dimensions of the expected event sensor
	 * @param contribution_ 	Contribution coefficient for a single event. The contribution value is multiplied
	 * 							by the maximum possible pixel value (255) to get the increment value.
	 * 							E.g. contribution value of 0.1 will increment a pixel value at a single event
	 * 							coordinates by 26.
	 * @param ignorePolarity_	Set ignore polarity option. All events are considered positive if enabled.
	 * @param neutralPotential	Neutral potential value. Neutral value is the default pixel value when decay is
	 * 							disabled and the value that pixels decay into when decay is enabled. The range for
	 * 							neutral potential value is [0.0; 1.0], where 1.0 stands for maximum possible
	 * 							potential - 255 in 8-bit pixel representation.
	 * @param decay_			Decay coefficient value. This value defines how fast pixel values decay to neutral
	 * 							value. The bigger the value the faster the pixel value will reach neutral value.
	 * 							Decay is applied before each frame generation. The range for decay value is [0.0; 1.0],
	 * 							where 0.0 will not apply any decay and 1.0 will apply maximum decay value resetting a
	 * 							pixel to neutral potential at each generation (default behavior).
	 */
	explicit EdgeMapAccumulator(const cv::Size &resolution, const float contribution_ = 0.25f,
		const bool ignorePolarity_ = true, const float neutralPotential = 0.f,
		const float decay_ = EdgeMapAccumulator::DECAY_FULL) :
		AccumulatorBase(resolution),
		ignorePolarity(ignorePolarity_),
		imageBuffer(resolution, CV_8UC1) {
		setEventContribution(contribution_);
		setNeutralPotential(neutralPotential);
		setDecay(decay_);
		imageBuffer = neutralByteValue;
	}

	/**
	 * Get the contribution coefficient for a single event. The contribution value is multiplied
	 * by the maximum possible pixel value (255) to get the increment value.
	 * E.g. contribution value of 0.1 will increment a pixel value at a single event
	 * coordinates by 26.
	 * @return		Contribution coefficient
	 * @deprecated  Use getEventContribution() method instead.
	 * @sa dv::EdgeMapAccumulator::getEventContribution
	 */
	[[deprecated("Use getEventContribution() method instead.")]] [[nodiscard]] float getContribution() const {
		return getEventContribution();
	}

	/**
	 * Set new contribution coefficient.
	 * @param contribution_ 	Contribution coefficient for a single event. The contribution value is multiplied
	 * 							by the maximum possible pixel value (255) to get the increment value.
	 * 							E.g. contribution value of 0.1 will increment a pixel value at a single event
	 * 							coordinates by 26.
	 * @deprecated Use setEventContribution() method instead.
	 * @sa dv::EdgeMapAccumulator::setEventContribution
	 */
	[[deprecated("Use setEventContribution() method instead.")]] void setContribution(const float contribution_) {
		setEventContribution(contribution_);
	}

	/**
	 * Get the contribution coefficient for a single event. The contribution value is multiplied
	 * by the maximum possible pixel value (255) to get the increment value.
	 * E.g. contribution value of 0.1 will increment a pixel value at a single event
	 * coordinates by 26.
	 * @return		Contribution coefficient
	 */
	[[nodiscard]] float getEventContribution() const {
		return contribution;
	}

	/**
	 * Set new contribution coefficient.
	 * @param contribution_ 	Contribution coefficient for a single event. The contribution value is multiplied
	 * 							by the maximum possible pixel value (255) to get the increment value.
	 * 							E.g. contribution value of 0.1 will increment a pixel value at a single event
	 * 							coordinates by 26.
	 */
	void setEventContribution(const float contribution_) {
		if (contribution_ < 0.f || contribution_ > 1.f) {
			throw std::invalid_argument("Contribution value should be in the range [0.0; 1.0]");
		}
		contribution  = contribution_;
		drawIncrement = static_cast<uint8_t>(std::ceil(static_cast<float>(maxByteValue) * contribution_));

		incrementLUT.clear();

		if (!ignorePolarity) {
			// LUT for decrement
			for (int i = 0; i <= maxByteValue; i++) {
				incrementLUT.push_back(static_cast<uint8_t>(std::max(i - drawIncrement, 0)));
			}
		}

		// LUT for increments
		for (int i = 0; i <= maxByteValue; i++) {
			incrementLUT.push_back(static_cast<uint8_t>(std::min<int32_t>(i + drawIncrement, maxByteValue)));
		}
	}

	/**
	 * Perform accumulation on given events.
	 * @param packet 	Event store containing event to be accumulated.
	 */
	void accumulate(const EventStore &packet) override {
		buffer.add(packet);
	}

	/**
	 * Generates the accumulation frame (potential surface) at the time of the
	 * last consumed event.
	 * The function writes the output image into the given `outFrame` argument.
	 * The output frame will contain data with type CV_8UC1.
	 *
	 * The function resets any events accumulated up to this function call.
	 * @param frame the frame to generate the image to
	 */
	[[nodiscard]] dv::Frame generateFrame() override {
		cv::Mat image;

		// Decay handling
		switch (decayMode) {
			case DecayMode::None:
				// No decay - just use buffered image from previous iteration
				imageBuffer.copyTo(image);
				break;
			case DecayMode::Full:
				// Full decay, allocate memory if needed and plainly reset to neutral value
				image = cv::Mat(shape_, CV_8UC1, neutralByteValue);
				break;
			case DecayMode::Decay:
				// Do decay with a lookup-table
				dv::runtime_assert(!decayLUT.empty(), "Decay lookup-table is empty, decay impossible");
				cv::LUT(imageBuffer, decayLUT, image);
				break;
		}

		if (ignorePolarity) {
			for (const dv::concepts::AddressableEvent auto &event : buffer) {
				dv::runtime_assert(0 <= event.y() && event.y() <= shape_.height, "event Y coordinate out of bounds");
				dv::runtime_assert(0 <= event.x() && event.x() <= shape_.width, "event X coordinate out of bounds");

				auto &imgVal = image.at<uint8_t>(event.y(), event.x());
				imgVal       = static_cast<uint8_t>(incrementLUT[imgVal]);
			}
		}
		else {
			for (const dv::concepts::AddressableEvent auto &event : buffer) {
				dv::runtime_assert(0 <= event.y() && event.y() <= shape_.height, "event Y coordinate out of bounds");
				dv::runtime_assert(0 <= event.x() && event.x() <= shape_.width, "event X coordinate out of bounds");

				auto &imgVal         = image.at<uint8_t>(event.y(), event.x());
				const size_t address = imgVal + (static_cast<size_t>(event.polarity()) * 256);
				imgVal               = static_cast<uint8_t>(incrementLUT[address]);
			}
		}

		const auto frameTimestamp = (buffer.isEmpty() ? -1 : buffer.getLowestTime());
		const auto frameExposure  = buffer.duration().count();

		// Image buffering can be skipped if we are in full decay mode which resets everything to neutral value at each
		// generation
		if (decayMode != DecayMode::Full) {
			// Save frame for decay
			image.copyTo(imageBuffer);
		}

		// Clear the buffer
		buffer = EventStore();

		return {frameTimestamp, frameExposure, 0, 0, image, dv::FrameSource::ACCUMULATION};
	}

	/**
	 * Clear the buffered events.
	 */
	void reset() {
		buffer      = EventStore();
		imageBuffer = cv::Mat(shape_, CV_8UC1);
		imageBuffer = neutralByteValue;
	}

	/**
	 * Accumulates the event store into the accumulator.
	 * @param store The event store to be accumulated.
	 * @return A reference to this EdgeMapAccumulator.
	 */
	EdgeMapAccumulator &operator<<(const EventStore &store) {
		accumulate(store);
		return *this;
	}

	/**
	 * Check whether ignore polarity option is set to true.
	 * @return True if the accumulator assumes all events as positive, false otherwise.
	 */
	[[nodiscard]] bool isIgnorePolarity() const {
		return ignorePolarity;
	}

	/**
	 * Set ignore polarity option. All events are considered positive if enabled.
	 * @param ignorePolarity_ True to enable ignore polarity option.
	 */
	void setIgnorePolarity(const bool ignorePolarity_) {
		EdgeMapAccumulator::ignorePolarity = ignorePolarity_;
		// Regenerate LUT
		setEventContribution(contribution);
	}

	/**
	 * Get the neutral potential value for the accumulator. The range for potential value is
	 * [0.0; 1.0], where 1.0 stands for maximum possible potential - 255 in 8-bit pixel representation.
	 * @return Neutral potential value in range [0.0; 1.0]
	 * @deprecated Use getNeutralPotential() method instead.
	 * @sa dv::EdgeMapAccumulator::getNeutralPotential
	 */
	[[deprecated("Use getNeutralPotential() method instead.")]] [[nodiscard]] float getNeutralValue() const {
		return getNeutralPotential();
	}

	/**
	 * Set the neutral potential value. The value should be in range 0.0 to 1.0, other values will be clamped
	 * to this range.
	 * @param neutralValue_ Neutral potential value in range [0.0; 1.0].
	 * @deprecated Use setNeutralPotential() method instead.
	 * @sa dv::EdgeMapAccumulator::setNeutralPotential
	 */
	[[deprecated("Use setNeutralPotential() method instead.")]] void setNeutralValue(const float neutralValue_) {
		setNeutralPotential(neutralValue_);
	}

	/**
	 * Get the neutral potential value for the accumulator. The range for potential value is
	 * [0.0; 1.0], where 1.0 stands for maximum possible potential - 255 in 8-bit pixel representation.
	 * @return Neutral potential value in range [0.0; 1.0]
	 */
	[[nodiscard]] float getNeutralPotential() const {
		return neutralValue;
	}

	/**
	 * Set the neutral potential value. The value should be in range 0.0 to 1.0, other values will be clamped
	 * to this range.
	 * @param neutralPotential Neutral potential value in range [0.0; 1.0].
	 */
	void setNeutralPotential(const float neutralPotential) {
		neutralValue     = std::clamp(neutralPotential, 0.f, 1.f);
		neutralByteValue = static_cast<uint8_t>(
			std::clamp<int>(static_cast<int>(neutralValue * static_cast<float>(maxByteValue)), 0, maxByteValue));
	}

	/**
	 * Get current decay value.
	 * @return Decay value.
	 */
	[[nodiscard]] float getDecay() const {
		return decay;
	}

	/**
	 * Set the decay value. Decay value is clamped to range of [0.0; 1.0].
	 * @param decay_ Decay value. Negative value disabled the decay.
	 */
	void setDecay(const float decay_) {
		decay = std::clamp(decay_, 0.f, 1.f);
		decayLUT.clear();

		// None and full decay modes does not require LUT, so we can exit early
		if (decay == DECAY_NONE) {
			decayMode = DecayMode::None;
			return;
		}
		else if (decay == DECAY_FULL) {
			decayMode = DecayMode::Full;
			return;
		}
		else {
			decayMode = DecayMode::Decay;
		}

		auto decayByteValue = static_cast<uint8_t>(static_cast<float>(maxByteValue) * std::min(decay, 1.f));

		for (int32_t i = 0; i <= maxByteValue; i++) {
			if (i == neutralByteValue) {
				// value is neutral
				decayLUT.push_back(neutralByteValue);
			}
			else if (i < neutralByteValue) {
				// Value is less than neutral, increase back to neutral
				decayLUT.push_back(static_cast<uint8_t>(std::clamp<int32_t>(i + decayByteValue, 0, neutralByteValue)));
			}
			else {
				// Value is more than neutral, decrease back to neutral
				decayLUT.push_back(
					static_cast<uint8_t>(std::clamp<int32_t>(i - decayByteValue, neutralByteValue, maxByteValue)));
			}
		}
	}
};

static_assert(dv::concepts::EventToFrameConverter<EdgeMapAccumulator, dv::EventStore>);

using PixelAccumulator [[deprecated("Please use dv::EdgeMapAccumulator class instead.")]] = EdgeMapAccumulator;

} // namespace dv

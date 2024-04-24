#pragma once

#include "../core.hpp"

namespace dv {
/**
 * An accumulator base that can be used to implement different types of accumulators.
 * Two provided implementations are the `dv::Accumulator` which is highly configurable
 * and provides numerous ways of generating a frame from events. Another implementation
 * is the `dv::EdgeMapAccumulator` which accumulates event in a histogram representation
 * with configurable contribution, but it is more efficient compared to generic
 * accumulator since it uses 8-bit unsigned integers as internal memory type.
 */
class AccumulatorBase {
protected:
	// output
	cv::Size shape_;

public:
	typedef std::shared_ptr<AccumulatorBase> SharedPtr;
	typedef std::unique_ptr<AccumulatorBase> UniquePtr;

	/**
	 * Accumulator constructor from known event camera sensor dimensions.
	 * @param shape 	Sensor dimensions
	 */
	explicit AccumulatorBase(const cv::Size &shape) : shape_(shape) {
	}

	/**
	 * Accumulate given event store packet into a frame.
	 * @param packet 	Event packet to be accumulated.
	 */
	virtual void accumulate(const EventStore &packet) = 0;

	/**
	 * Get the image dimensions expected by the accumulator.
	 * @return 			Image dimensions
	 */
	[[nodiscard]] const cv::Size &getShape() const {
		return shape_;
	}

	/**
	 * Generates the accumulation frame (potential surface) at the time of the
	 * last consumed event.
	 * The function returns an OpenCV frame to work
	 * with.
	 * @return An OpenCV frame containing the accumulated potential surface.
	 */
	[[nodiscard]] virtual dv::Frame generateFrame() = 0;

	/**
	 * Output stream operator support for frame generation.
	 * @param mat 	Output image
	 * @return 		Output image
	 */
	dv::Frame &operator>>(dv::Frame &mat) {
		mat = generateFrame();
		return mat;
	}

	/**
	 * Accumulate the given packet.
	 * @param packet 	Input event packet.
	 */
	inline void accept(const EventStore &packet) {
		accumulate(packet);
	}

	virtual ~AccumulatorBase() = default;
};

} // namespace dv

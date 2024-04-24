#pragma once

#include "../core/concepts.hpp"
#include "../core/frame.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "../measurements/depth.hpp"
#include "linear_transformer.hpp"
#include "pixel_motion_predictor.hpp"

namespace dv::kinematics {

template<class Accumulator = dv::EdgeMapAccumulator, class PixelPredictor = kinematics::PixelMotionPredictor>
requires dv::concepts::EventToFrameConverter<Accumulator, dv::EventStore>
class MotionCompensator {
public:
	struct Info {
		bool imageCompensated    = false;
		bool depthAvailable      = false;
		bool transformsAvailable = false;
		int64_t depthTime        = -1LL;
		int64_t generationTime   = -1LL;

		size_t inputEventCount       = 0ULL;
		size_t accumulatedEventCount = 0ULL;
	};

private:
	PixelPredictor predictor;
	dv::kinematics::LinearTransformerf transformer;
	std::unique_ptr<Accumulator> accumulator;
	std::map<int64_t, float> depths;
	float constantDepth = 3.f;
	dv::EventStore eventBuffer;

	int64_t storageDuration      = 5000000LL;
	const int64_t samplingPeriod = 200LL;

	MotionCompensator::Info info;

public:
	/**
	 * Return an info class instance containing motion compensator state for the algorithm iteration. The info object
	 * contains debug information about the execution of the motion compensator.
	 * @return
	 */
	[[nodiscard]] const Info &getInfo() const {
		return info;
	}

	/**
	 * Push camera pose measurement.
	 * @param transform 	Transform representing camera pose in some fixed reference frame (e.g. World coordinates).
	 */
	void accept(const Transformationf &transform) {
		transformer.pushTransformation(transform);
	}

	/**
	 * Scene depth measurement in meters.
	 * @param timeDepth 	A pair containing measured depth into the scene and a timestamp at when the measurement
	 * 						was performed.
	 */
	void accept(const dv::measurements::Depth &timeDepth) {
		depths.insert(std::make_pair(timeDepth.mTimestamp, timeDepth.mDepth));
		int64_t maxTime = timeDepth.mTimestamp - storageDuration;
		std::erase_if(depths, [maxTime](const std::pair<int64_t, float> &val) {
			return val.first < maxTime;
		});
	}

	/**
	 * Push event camera input.
	 * @param events 	Pixel brightness changes from an event camera.
	 */
	void accept(const dv::EventStore &events) {
		eventBuffer.add(events);
	}

	/**
	 * Push event camera input.
	 * @param event 	Pixel brightness change from an event camera.
	 */
	void accept(const dv::Event &event) {
		eventBuffer.push_back(event);
	}

	/**
	 * Generate the motion compensated events contained in the buffer.
	 * @param generationTime Provide a timestamp to which point in time the motion compensator compensates into,
	 *						negative values will cause the function to use highest timestamp value in the event
	 *						buffer.
	 * @return				Motion compensated events.
	 */
	dv::EventStore generateEvents(const int64_t generationTime = -1) {
		dv::EventStore compensatedEvents
			= generateEventsAt(generationTime > 0 ? generationTime : eventBuffer.getHighestTime());

		reset();

		return compensatedEvents;
	}

	/**
	 * Generate the motion compensated frame output and reset the events contained in the buffer.
	 * @param generationTime 	Provide a timestamp to which point in time the motion compensator compensates into,
	 * 							negative values will cause the function to use highest timestamp value in the event
	 * 							buffer.
	 * @return 					Motion compensated frame.
	 */
	dv::Frame generateFrame(const int64_t generationTime = -1) {
		dv::Frame output = generateFrameAt(generationTime > 0 ? generationTime : eventBuffer.getHighestTime());

		reset();

		return output;
	}

	/**
	 * Clear the event buffer.
	 */
	void reset() {
		eventBuffer = dv::EventStore();
	}

	/**
	 * Accept the event data using the stream operator.
	 * @param store 	Input event store.
	 * @return 			Reference to current object instance.
	 */
	MotionCompensator &operator<<(const dv::EventStore &store) {
		accept(store);
		return *this;
	}

	/**
	 * Accept the event data using the stream operator.
	 * @param store 	Input event.
	 * @return 			Reference to current object instance.
	 */
	MotionCompensator &operator<<(const dv::Event &event) {
		accept(event);
		return *this;
	}

	/**
	 * Output stream operator which generates a frame.
	 * @param image 	Motion compensated frame.
	 * @return 			Motion compensated frame.
	 */
	dv::Frame &operator>>(dv::Frame &image) {
		image = generateFrame();
		return image;
	}

	/**
	 * Construct a motion compensator instance with custom accumulator.
	 * @param cameraGeometry 	Camera geometry class instance containing intrinsic calibration of the camera sensor.
	 * @param accumulator_ 		Accumulator instance to be used to accumulate events.
	 */
	MotionCompensator(
		const camera::CameraGeometry::SharedPtr &cameraGeometry, std::unique_ptr<Accumulator> accumulator_) :
		predictor(cameraGeometry),
		transformer(1000),
		accumulator(std::move(accumulator_)) {
	}

	/**
	 * Construct a motion compensator instance with default accumulator. Default accumulator is a
	 * `dv::EdgeMapAccumulator` with default parameters.
	 * @param cameraGeometry 	Camera geometry class instance containing intrinsic calibration of the camera sensor.
	 */
	explicit MotionCompensator(const camera::CameraGeometry::SharedPtr &cameraGeometry) :
		predictor(cameraGeometry),
		transformer(1000),
		accumulator(std::make_unique<Accumulator>(cameraGeometry->getResolution())) {
	}

	/**
	 * Construct a motion compensator with no known calibration. This assumes that the camera is an ideal
	 * pinhole camera sensor (no distortion) with focal length equal to camera sensor width in pixels and central point
	 * is the exact geometrical center of the pixel array.
	 * @param sensorDimensions 	Camera sensor resolution.
	 */
	explicit MotionCompensator(const cv::Size &sensorDimensions) :
		predictor(std::make_shared<dv::camera::CameraGeometry>(static_cast<float>(sensorDimensions.width),
			static_cast<float>(sensorDimensions.width), static_cast<float>(sensorDimensions.width) / 2.f,
			static_cast<float>(sensorDimensions.height) / 2.f, sensorDimensions)),
		transformer(1000),
		accumulator(std::make_unique<Accumulator>(sensorDimensions)) {
	}

	/**
	 * Get currently assumed constant depth value. It is used if no depth measurements are provided.
	 * @return 	Currently used aistance to the scene (depth).
	 * @sa setConstantDepth
	 */
	[[nodiscard]] float getConstantDepth() const {
		return constantDepth;
	}

	/**
	 * Set constant depth value that is assumed if no depth measurement is passed using
	 * `accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is just a
	 * reasonable guess.
	 * @param depth 	Distance to the scene (depth).
	 * @throws InvalidArgument Exception is thrown if a negative depth value is passed.
	 */
	void setConstantDepth(const float depth) {
		if (depth < 0.f) {
			throw dv::exceptions::InvalidArgument<float>(
				"Negative constant depth provided, please provide positive value only", depth);
		}

		constantDepth = depth;
	}

private:
	/**
	 * Generate a sequence of transformations at a fixed period (`samplingPeriod`) with an additional overhead
	 * transform before and after the given interval.
	 * @param from 		Start of the interest interval.
	 * @param to		End of the interest interval.
	 * @return 			Transformer with resampled transformations.
	 */
	[[nodiscard]] dv::kinematics::LinearTransformerf generateTransforms(const int64_t from, const int64_t to) {
		// (MC) Transformations generation
		const int64_t interval                = (to - from) / 2LL;
		static constexpr int64_t minExtraTime = 5000LL;
		static constexpr int64_t maxExtraTime = 20000LL;
		const int64_t transExtraTime          = std::clamp(interval, minExtraTime, maxExtraTime);
		const int64_t transformationsFrom     = from - transExtraTime;
		const int64_t transformationsTo       = to + transExtraTime;

		// Slice the exact period
		auto slicedTransforms = transformer.getTransformsBetween(transformationsFrom, transformationsTo);

		// Densify using resampling by interpolation
		return slicedTransforms.resampleTransforms(samplingPeriod);
	}

	/**
	 * Apply motion compensation to event store and project all event into the target transformation.
	 * @param events 		Input events.
	 * @param transforms 	Transformer containing the fine grained trajectory of the camera motion.
	 * @param target 		Target position of the camera to be projected into.
	 * @param depth			Scene depth to be assumed for the calculations.
	 * @return 				Motion compensated events at the target camera pose.
	 */
	[[nodiscard]] dv::EventStore compensateEvents(const dv::EventStore &events,
		const dv::kinematics::LinearTransformerf &transforms, const dv::kinematics::Transformationf &target,
		const float depth) {
		dv::EventStore compensated;
		auto prev  = transforms.cbegin();
		auto T_CW1 = target.inverse().getTransform();
		for (auto next = transforms.cbegin() + 1; next != transforms.cend(); next++, prev++) {
			dv::EventStore thinSlice = events.sliceTime(prev->getTimestamp() + 1, next->getTimestamp() + 1);
			auto deltaT              = dv::kinematics::Transformationf(0, T_CW1 * prev->getTransform());
			compensated.add(predictor.predictEvents(thinSlice, deltaT, depth));
		}
		return compensated;
	}

	/**
	 * Generate compensated events at a given timestamp.
	 * @param timestamp 	time to compensate events at.
	 * @return 				A motion compensated events at given time point.
	 */
	[[nodiscard]] dv::EventStore generateEventsAt(const int64_t timestamp) {
		info                = MotionCompensator::Info();
		info.generationTime = timestamp;

		dv::EventStore events = eventBuffer;
		info.inputEventCount  = eventBuffer.size();

		float depth;
		if (depths.empty()) {
			depth = constantDepth;
		}
		else if (auto closestDepth = depths.upper_bound(timestamp); closestDepth != depths.end()) {
			depth               = closestDepth->second;
			info.depthAvailable = true;
			info.depthTime      = closestDepth->first;
		}
		else {
			depth               = depths.rbegin()->second;
			info.depthAvailable = true;
			info.depthTime      = depths.rbegin()->first;
		}

		auto interpolatedTransforms = generateTransforms(events.getLowestTime(), events.getHighestTime());
		if (auto finalT = interpolatedTransforms.getTransformAt(timestamp)) {
			events                   = compensateEvents(events, interpolatedTransforms, *finalT, depth);
			info.transformsAvailable = true;
		}

		return events;
	}

	/**
	 * Generate a frame at a given timestamp.
	 * @param timestamp 		Time to generate frame at.
	 * @return 					A motion compensated frame at given time point.
	 */
	[[nodiscard]] dv::Frame generateFrameAt(const int64_t timestamp) {
		dv::FrameSource source;
		dv::EventStore events;

		events = generateEventsAt(timestamp);

		if (info.transformsAvailable) {
			info.imageCompensated = true;
			source                = dv::FrameSource::MOTION_COMPENSATION;
		}
		else {
			source = dv::FrameSource::ACCUMULATION;
		}

		info.accumulatedEventCount = events.size();

		accumulator->accept(events);
		dv::Frame frame = accumulator->generateFrame();

		// Override the frame source to reflect the use of motion compensation
		frame.source = source;
		return frame;
	}

public:
	dv::EventStore &operator>>(dv::EventStore &out) {
		out = generateEvents();
		return out;
	}
};

static_assert(concepts::EventToEventConverter<dv::kinematics::MotionCompensator<>, dv::EventStore>);
static_assert(concepts::EventToFrameConverter<dv::kinematics::MotionCompensator<>, dv::EventStore>);
static_assert(concepts::Accepts<dv::kinematics::MotionCompensator<>, dv::measurements::Depth>);
static_assert(concepts::Accepts<dv::kinematics::MotionCompensator<>, dv::kinematics::Transformationf>);
static_assert(concepts::SupportsConstantDepth<dv::kinematics::MotionCompensator<>>);

} // namespace dv::kinematics

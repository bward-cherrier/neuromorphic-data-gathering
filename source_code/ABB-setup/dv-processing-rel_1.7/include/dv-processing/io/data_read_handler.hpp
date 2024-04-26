#pragma once

#include "../core/core.hpp"
#include "../core/frame.hpp"
#include "../data/imu_base.hpp"
#include "../data/trigger_base.hpp"

#include <functional>
#include <optional>
#include <variant>

namespace dv::io {

/**
 * Read handler that can handle all supported types in MonoCameraRecording.
 */
struct DataReadHandler {
	enum class OutputFlag {
		EndOfFile,
		Continue
	};

	/**
	 * Event handler that is going to be called on each arriving event batch.
	 */
	std::optional<std::function<void(const dv::EventStore &)>> mEventHandler = std::nullopt;

	/**
	 * Frame handler that is called on each arriving frame.
	 */
	std::optional<std::function<void(const dv::Frame &)>> mFrameHandler = std::nullopt;

	/**
	 * IMU data handler that is going to be called on each arriving imu data batch.
	 */
	std::optional<std::function<void(const dv::cvector<dv::IMU> &)>> mImuHandler = std::nullopt;

	/**
	 * Trigger data handler that is going to be called on each arriving trigger data batch.
	 */
	std::optional<std::function<void(const dv::cvector<dv::Trigger> &)>> mTriggersHandler = std::nullopt;

	/**
	 * A handler for output flags that can indicate some file behaviour, e.g. end-of-file.
	 */
	std::optional<std::function<void(const OutputFlag)>> mOutputFlagHandler = std::nullopt;

	/**
	 * Is end of file reached.
	 */
	bool eof = false;

	/**
	 * Timestamp holding latest seek position of the recording
	 */
	int64_t seek = -1;

	/**
	 * Internal call to handle input data
	 * @param events
	 */
	void operator()(const dv::EventStore &events) {
		seek = events.getLowestTime();
		if (mEventHandler.has_value()) {
			mEventHandler.value()(events);
		}
	}

	/**
	 * Internal call to handle input data
	 * @param frame
	 */
	void operator()(const dv::Frame &frame) {
		seek = frame.timestamp;
		if (mFrameHandler.has_value()) {
			mFrameHandler.value()(frame);
		}
	}

	/**
	 * Internal call to handle input data
	 * @param triggers
	 */
	void operator()(const dv::cvector<dv::Trigger> &triggers) {
		if (!triggers.empty()) {
			seek = triggers.front().timestamp;
		}
		if (mTriggersHandler.has_value()) {
			mTriggersHandler.value()(triggers);
		}
	}

	/**
	 * Internal call to handle input data
	 * @param imu
	 */
	void operator()(const dv::cvector<dv::IMU> &imu) {
		if (!imu.empty()) {
			seek = imu.front().timestamp;
		}
		if (mImuHandler.has_value()) {
			mImuHandler.value()(imu);
		}
	}

	/**
	 * Internal call to handle input data
	 * @param flag
	 */
	void operator()(const OutputFlag flag) {
		if (mOutputFlagHandler.has_value()) {
			mOutputFlagHandler.value()(flag);
		}
		if (flag == OutputFlag::EndOfFile) {
			eof = true;
		}
	}
};

using DataReadVariant = std::variant<dv::EventStore, dv::Frame, dv::cvector<dv::IMU>, dv::cvector<dv::Trigger>,
	DataReadHandler::OutputFlag>;

} // namespace dv::io

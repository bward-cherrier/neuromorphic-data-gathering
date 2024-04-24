#pragma once

#include "../core/core.hpp"
#include "../data/cvector.hpp"
#include "../data/event_base.hpp"
#include "../data/frame_base.hpp"
#include "../data/imu_base.hpp"
#include "../data/trigger_base.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>

namespace dv::io {

/**
 * Camera input base class to abstract live camera and recorded files with a common interface.
 */
class CameraInputBase {
public:
	/**
	 * Parse and retrieve next event batch.
	 * @return 		Event batch or `std::nullopt` if no events were received since last read.
	 */
	[[nodiscard]] virtual std::optional<dv::EventStore> getNextEventBatch() = 0;

	/**
	 * Parse and retrieve next frame.
	 * @return 		Frame or `std::nullopt` if no frames were received since last read.
	 */
	[[nodiscard]] virtual std::optional<dv::Frame> getNextFrame() = 0;

	/**
	 * Parse and retrieve next IMU data batch.
	 * @return 		IMU data batch or `std::nullopt` if no IMU data was received since last read.
	 */
	[[nodiscard]] virtual std::optional<dv::cvector<dv::IMU>> getNextImuBatch() = 0;

	/**
	 * Parse and retrieve next trigger data batch.
	 * @return 		Trigger data batch or `std::nullopt` if no triggers were received since last read.
	 */
	[[nodiscard]] virtual std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch() = 0;

	/**
	 * Get event stream resolution.
	 * @return 		Event stream resolution, `std::nullopt` if event stream is unavailable.
	 */
	[[nodiscard]] virtual std::optional<cv::Size> getEventResolution() const = 0;

	/**
	 * Retrieve frame stream resolution.
	 * @return 		Frame stream resolution or `std::nullopt` if the frame stream is not available.
	 */
	[[nodiscard]] virtual std::optional<cv::Size> getFrameResolution() const = 0;

	/**
	 * Check whether event stream is available.
	 * @return 		True if event stream is available, false otherwise.
	 */
	[[nodiscard]] virtual bool isEventStreamAvailable() const = 0;

	/**
	 * Check whether frame stream is available.
	 * @return 		True if frame stream is available, false otherwise.
	 */
	[[nodiscard]] virtual bool isFrameStreamAvailable() const = 0;

	/**
	 * Check whether IMU data is available.
	 * @return True if IMU data stream is available, false otherwise.
	 */
	[[nodiscard]] virtual bool isImuStreamAvailable() const = 0;

	/**
	 * Check whether trigger data is available.
	 * @return True if trigger data stream is available, false otherwise.
	 */
	[[nodiscard]] virtual bool isTriggerStreamAvailable() const = 0;

	/**
	 * Get camera name, which is a combination of the camera model and the serial number.
	 * @return 		String containing the camera model and serial number separated by an underscore character.
	 */
	[[nodiscard]] virtual std::string getCameraName() const = 0;

	/**
	 * Check whether input data streams are still available. For a live camera this should check whether device is still
	 * connected and functioning, while for a recording file this should check whether end of stream was reached using
	 * sequential reads.
	 * @return 		True if data read is possible, false otherwise.
	 */
	[[nodiscard]] virtual bool isRunning() const = 0;
};

} // namespace dv::io

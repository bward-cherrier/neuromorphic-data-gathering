#pragma once

#include "../core/core.hpp"

namespace dv::io {

/**
 * Output reader base class defining API interface for writing camera data into an IO resource.
 */
class CameraOutputBase {
public:
	/**
	 * Write event data into the output.
	 * @param events Write events into the output.
	 */
	virtual void writeEvents(const dv::EventStore &events) = 0;

	/**
	 * Write a frame into the output.
	 * @param frame Write a frame into the output.
	 */
	virtual void writeFrame(const dv::Frame &frame) = 0;

	/**
	 * Write imu data into the output.
	 * @param imu Write imu into the output.
	 */
	virtual void writeIMU(const dv::cvector<dv::IMU> &imu) = 0;

	/**
	 * Write trigger data into the output.
	 * @param triggers Write trigger into the output.
	 */
	virtual void writeTriggers(const dv::cvector<dv::Trigger> &triggers) = 0;

	/**
	 * Retrieve camera name of this writer output instance.
	 * @return Configured camera name.
	 */
	[[nodiscard]] virtual std::string getCameraName() const = 0;
};

} // namespace dv::io

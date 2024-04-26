#pragma once

#include "camera_capture.hpp"

namespace dv::io {

class StereoCapture {
private:
	/**
	 * Performs synchronization of the stereo camera setup.
	 * @param master 		Camera capture instance that has generates synchronization signal.
	 * @param secondary 	Camera capture instance that receives synchronization signal.
	 * @param timeout 		An exception is thrown if synchronization doesn't complete within given time
	 * 						period in microseconds.
	 */
	static void synchronizeStereo(CameraCapture &master, CameraCapture &secondary, const int64_t timeout) {
		// Wait a bit before resetting timestamp to allow old data to be discarded.
		std::this_thread::sleep_for(std::chrono::milliseconds{500});

		secondary.initState = CameraCapture::InitialState::WAIT_FOR_RESET;
		master.sendTimestampReset();

		int64_t timeNow = dv::now();
		while ((master.initState != CameraCapture::InitialState::RUNNING)
			   || (secondary.initState != CameraCapture::InitialState::RUNNING)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			if ((dv::now() - timeNow) > timeout) {
				throw dv::exceptions::RuntimeError("Stereo synchronization timeout reached");
			}
		}

		secondary.setTimestampOffset(master.getTimestampOffset());
	}

public:
	CameraCapture left;
	CameraCapture right;

	/**
	 * Open a stereo camera setup consisting of two cameras. Finds the devices connected to the system and performs
	 * timestamp synchronization on them.
	 * @param leftName 			Left camera name.
	 * @param rightName			Right camera name.
	 * @param synchronizationTimeout	Timeout duration for synchronization
	 * @throws RuntimeError		Exception if both cameras are master (missing sync cable between cameras
	 * 							is the most likely reason).
	 * @throws RuntimeError		Exception is thrown if cameras fails to synchronize within given timeout duration.
	 */
	StereoCapture(const std::string &leftName, const std::string &rightName,
		const dv::Duration &synchronizationTimeout = dv::Duration(1'000'000)) :
		left(leftName, CameraCapture::CameraType::Any, false),
		right(rightName, CameraCapture::CameraType::Any, false) {
		if (left.isMasterCamera() && right.isMasterCamera()) {
			throw dv::exceptions::RuntimeError(
				"StereoCapture: both cameras seem to be master, please make sure the sync cable is connected.");
		}

		if (left.isMasterCamera()) {
			synchronizeStereo(left, right, synchronizationTimeout.count());
		}
		else {
			synchronizeStereo(right, left, synchronizationTimeout.count());
		}
	}
};

} // namespace dv::io

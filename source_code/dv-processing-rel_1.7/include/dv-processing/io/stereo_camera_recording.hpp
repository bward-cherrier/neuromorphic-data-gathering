#pragma once

#include "mono_camera_recording.hpp"

namespace dv::io {

namespace fs = std::filesystem;

class StereoCameraRecording {
private:
	std::shared_ptr<ReadOnlyFile> mReader = nullptr;
	MonoCameraRecording mLeftCamera;

	MonoCameraRecording mRightCamera;

public:
	/**
	 * Create a reader for stereo camera recording. Expects at least one stream from two cameras available.
	 * Prior knowledge of stereo setup is required, otherwise it is not possible to differentiate between
	 * left and right cameras. This is just a convenience class that gives access to distinguished data streams
	 * in the recording.
	 * @param aedat4Path 			Path to the aedat4 file.
	 * @param leftCameraName 		Name of the left camera.
	 * @param rightCameraName 		Name of the right camera.
	 */
	StereoCameraRecording(
		const fs::path &aedat4Path, const std::string &leftCameraName, const std::string &rightCameraName) :
		mReader(std::make_shared<ReadOnlyFile>(aedat4Path, dv::io::support::defaultTypeResolver)),
		mLeftCamera(mReader, leftCameraName),
		mRightCamera(mReader, rightCameraName) {
	}

	/**
	 * Access the left camera.
	 * @return 		A reference to the left camera reader.
	 */
	MonoCameraRecording &getLeftReader() {
		return mLeftCamera;
	}

	/**
	 * Access the right camera.
	 * @return 		A reference to the right camera reader.
	 */
	MonoCameraRecording &getRightReader() {
		return mRightCamera;
	}
};

} // namespace dv::io

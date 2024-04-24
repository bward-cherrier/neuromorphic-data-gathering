#include <dv-processing/io/camera_capture.hpp>

#include <chrono>

int main() {
	using namespace std::chrono_literals;

	// Open a Davis camera
	dv::io::CameraCapture capture("", dv::io::CameraCapture::CameraType::DAVIS);

	// Enable frame auto-exposure (default behavior)
	capture.enableDavisAutoExposure();
	// Disable auto-exposure, set frame exposure (here 10ms)
	capture.setDavisExposureDuration(dv::Duration(10ms));
	// Read current frame exposure duration value
	std::optional<dv::Duration> duration = capture.getDavisExposureDuration();
	// Set frame interval duration (here 33ms for ~30FPS)
	capture.setDavisFrameInterval(dv::Duration(33ms));
	// Read current frame interval duration value
	std::optional<dv::Duration> interval = capture.getDavisFrameInterval();

	return 0;
}

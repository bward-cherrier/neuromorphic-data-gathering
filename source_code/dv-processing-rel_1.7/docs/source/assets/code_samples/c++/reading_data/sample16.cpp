#include <dv-processing/io/camera_capture.hpp>

int main() {
	// Open a DVXplorer camera
	dv::io::CameraCapture capture("", dv::io::CameraCapture::CameraType::DVS);

	// Configure event sensitivity to default. Other sensitivities available: VeryLow, Low, High, VeryHigh
	capture.setDVSBiasSensitivity(dv::io::CameraCapture::BiasSensitivity::Default);

	// Configure event-frame readouts per second (here variable 5000 FPS, the default value)
	// See detailed API documentation for other available values
	capture.setDVXplorerEFPS(dv::io::CameraCapture::DVXeFPS::EFPS_VARIABLE_5000);

	// Enable global hold setting (already the default)
	capture.setDVSGlobalHold(true);
	// Disable global reset setting (already the default)
	capture.setDVXplorerGlobalReset(false);

	return 0;
}

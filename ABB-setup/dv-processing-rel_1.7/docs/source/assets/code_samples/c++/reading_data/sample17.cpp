#include <dv-processing/io/camera_capture.hpp>

int main() {
	// Open a Davis camera
	dv::io::CameraCapture capture("", dv::io::CameraCapture::CameraType::DAVIS);

	// Setting camera readout to events and frames (default). Other modes available: EventsOnly, FramesOnly
	capture.setDavisReadoutMode(dv::io::CameraCapture::DavisReadoutMode::EventsAndFrames);
	// Configure frame output mode to color (default), only on COLOR cameras. Other mode available: Grayscale
	capture.setDavisColorMode(dv::io::CameraCapture::DavisColorMode::Color);

	return 0;
}

#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/io/stereo_camera_writer.hpp>
#include <dv-processing/io/stereo_capture.hpp>

#include <CLI/CLI.hpp>

#include <csignal>

static std::atomic<bool> keepRunning(true);

static void handleShutdown(int) {
	keepRunning.store(false);
}

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	// CLI argument placeholders
	std::string aedat4Path;
	std::string cameraName;

	// Install signal handlers for a clean shutdown
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	// CLI argument handling
	CLI::App app{"Command-line single camera data recorder"};
	app.add_option("-o,--output-file", aedat4Path, "Path to an output aedat4 file for writing.")
		->required()
		->check(CLI::NonexistentPath);
	app.add_option("-c,--camera-name", cameraName,
		"Camera name (e.g. DVXplorer_DXA00093). The application will open any supported camera if no camera name is "
		"provided.");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Open the camera; the camera name can empty as well.
	dv::io::CameraCapture capture(cameraName);

	// Provide the opened camera name; Especially useful when no camera name is provided in the arguments.
	std::cout << "Camera [" << capture.getCameraName() << "] has been opened!" << std::endl;

	// Create the file writer instance
	dv::io::MonoCameraWriter writer(aedat4Path, capture);

	// Record the data by handling the inputs, exits when SIGINT (or Ctrl+C in terminal) is received
	std::cout << "Starting the recording!" << std::endl;
	while (keepRunning && capture.isRunning()) {
		// For each of the available streams - try reading a packet and write immediately to a file.
		if (capture.isEventStreamAvailable()) {
			if (const auto &events = capture.getNextEventBatch(); events.has_value()) {
				writer.writeEvents(*events);
			}
		}
		if (capture.isFrameStreamAvailable()) {
			if (const auto &frame = capture.getNextFrame(); frame.has_value()) {
				writer.writeFrame(*frame);
			}
		}
		if (capture.isImuStreamAvailable()) {
			if (const auto &imu = capture.getNextImuBatch(); imu.has_value()) {
				writer.writeImuPacket(*imu);
			}
		}
		if (capture.isTriggerStreamAvailable()) {
			if (const auto &triggers = capture.getNextTriggerBatch(); triggers.has_value()) {
				writer.writeTriggerPacket(*triggers);
			}
		}
	}

	// The recording required a tailing data table which is going to be written when MonoCameraWriter
	// goes out of scope.
	std::cout << "Finalizing the recording!" << std::endl;
	return EXIT_SUCCESS;
}

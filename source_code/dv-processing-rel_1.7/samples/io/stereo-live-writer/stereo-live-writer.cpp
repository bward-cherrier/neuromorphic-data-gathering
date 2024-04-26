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
	std::string leftName;
	std::string rightName;

	// Install signal handlers for a clean shutdown
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	// CLI argument handling
	CLI::App app{"Command-line stereo camera setup recorder"};
	app.add_option("-o,--output-file", aedat4Path, "Path to an output aedat4 file to be writter.")
		->required()
		->check(CLI::NonexistentPath);
	app.add_option("-l,--left-name", leftName, "Left camera name (e.g. DVXplorer_DXA00093).")->required();
	app.add_option("-r,--right-name", rightName, "Right camera name (e.g. DVXplorer_DXA00093).")->required();
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Open the cameras
	dv::io::StereoCapture stereo(leftName, rightName);

	// Use event only configurations for this sample
	const auto leftConfig = dv::io::MonoCameraWriter::EventOnlyConfig(
		stereo.left.getCameraName(), stereo.left.getEventResolution().value());

	const auto rightConfig = dv::io::MonoCameraWriter::EventOnlyConfig(
		stereo.right.getCameraName(), stereo.right.getEventResolution().value());

	// Create the file writer instance
	dv::io::StereoCameraWriter writer(aedat4Path, leftConfig, rightConfig);

	// Above configuration is customized, there is convenience method that can detect and initialize to capture
	// all available data steams. In that case custom configuration is not required. Writer instance can inspect the
	// capabilities by passing the stereo capture instance directly to constructor, like so:
	// dv::io::StereoCameraWriter writer(aedat4Path, stereo);

	// Connect the appropriate data streams handlers to write into the files
	dv::io::DataReadHandler leftHandler;
	leftHandler.mEventHandler = [&writer](const dv::EventStore &events) {
		writer.left.writeEvents(events);
	};
	if (stereo.left.isFrameStreamAvailable()) {
		leftHandler.mFrameHandler = [&writer](const dv::Frame &frame) {
			writer.left.writeFrame(frame);
		};
	}

	dv::io::DataReadHandler rightHandler;
	rightHandler.mEventHandler = [&writer](const dv::EventStore &events) {
		writer.right.writeEvents(events);
	};
	if (stereo.right.isFrameStreamAvailable()) {
		rightHandler.mFrameHandler = [&writer](const dv::Frame &frame) {
			writer.right.writeFrame(frame);
		};
	}

	// Record the data by handling the inputs, exits when SIGINT (or Ctrl+C in terminal) is received
	std::cout << "Starting the recording!" << std::endl;
	while (keepRunning) {
		if (!stereo.left.handleNext(leftHandler)) {
			break;
		}
		if (!stereo.right.handleNext(rightHandler)) {
			break;
		}
	}

	// The recording required a tailing data table which is going to be written when StereoCameraWriter
	// goes out of scope.
	std::cout << "Finalizing the recording!" << std::endl;
	return EXIT_SUCCESS;
}

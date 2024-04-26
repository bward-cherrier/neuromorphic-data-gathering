#include <dv-processing/core/frame.hpp>
#include <dv-processing/features/mean_shift_tracker.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/visualization/events_visualizer.hpp>

#include <CLI/CLI.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	std::string aedat4Path;

	CLI::App app{"Command-line for mean shift tracker script."};
	app.add_option("-i,--input", aedat4Path, "Path to an input aedat4 file.")->required()->check(CLI::ExistingFile);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	dv::io::MonoCameraRecording reader(aedat4Path);
	auto resolution = reader.getEventResolution().value();

	// Initialize a preview window
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Initialize a slicer
	dv::EventStreamSlicer slicer;

	// Bandwidth value defining the size of the search window in which updated track location will be searched
	const int bandwidth = 20;

	// Time window used for the normalized time surface computation. In this case we take the last 50ms of events and
	// compute a normalized time surface over them
	const dv::Duration timeWindow = 50ms;

	// Initialize a mean shift tracker
	dv::features::MeanShiftTracker meanShift = dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow);

	dv::visualization::EventVisualizer visualizer(resolution);

	// Register a callback every 33 milliseconds
	slicer.doEveryTimeInterval(33ms, [&](const dv::EventStore &events) {
		// feed data to mean shift tracker and run track update
		meanShift.accept(events);
		auto tracks = meanShift.runTracking();

		if (!tracks) {
			return;
		}
		// visualize mean shift tracks
		auto preview = visualizer.generateImage(events);
		for (const auto &track : tracks->keypoints) {
			const auto x = static_cast<int>(track.pt.x());
			const auto y = static_cast<int>(track.pt.y());
			cv::drawMarker(preview, cv::Point2i(x, y), dv::visualization::colors::red, cv::MARKER_CROSS, 20, 2);
		}

		cv::imshow("Preview", preview);
		cv::waitKey(100);
	});

	// Create a handler instance for storing lambda callback functions for each incoming type
	dv::io::DataReadHandler handler;

	handler.mEventHandler = [&](const dv::EventStore &store) {
		slicer.accept(store);
	};

	reader.run(handler);

	return EXIT_SUCCESS;
}

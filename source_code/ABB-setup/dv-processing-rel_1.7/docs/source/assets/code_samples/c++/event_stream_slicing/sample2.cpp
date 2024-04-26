#include <dv-processing/core/stereo_event_stream_slicer.hpp>
#include <dv-processing/io/stereo_capture.hpp>

int main() {
	using namespace std::chrono_literals;

	// Discover connected camera to the system
	const auto cameras = dv::io::discoverDevices();
	if (cameras.size() < 2) {
		throw dv::exceptions::RuntimeError("Unable to discover two cameras");
	}

	// Open the cameras, just use first detected cameras
	dv::io::StereoCapture stereo(cameras[0], cameras[1]);

	// Initialize a stereo stream slicer
	dv::StereoEventStreamSlicer slicer;

	// Register a job to be performed every 33 milliseconds
	slicer.doEveryTimeInterval(33ms, [](const dv::EventStore &leftEvents, const dv::EventStore &rightEvents) {
		// Print durations for time-based slicing callback
		std::cout << fmt::format(
			"* Received events with duration: left[{}] - right[{}]", leftEvents.duration(), rightEvents.duration())
				  << std::endl;
	});

	// Register a job to be performed every 1000 events
	slicer.doEveryNumberOfEvents(
		// Here we receive events from two camera, time-synchronized
		1000, [](const dv::EventStore &leftEvents, const dv::EventStore &rightEvents) {
			// Print event store sizes for number based slicing callback
			std::cout << fmt::format("# Received events in number-based slicing, counts: left[{}] - right[{}]",
				leftEvents.size(), rightEvents.size())
					  << std::endl;
		});

	// Continue the loop while both cameras are connected
	while (stereo.left.isRunning() && stereo.right.isRunning()) {
		// Initialize
		dv::EventStore left, right;

		// Handle left camera events
		if (const auto batch = stereo.left.getNextEventBatch(); batch.has_value()) {
			left = *batch;
		}

		// Handle right camera events
		if (const auto batch = stereo.right.getNextEventBatch(); batch.has_value()) {
			right = *batch;
		}

		// Pass all events into the slicer
		slicer.accept(left, right);
	}

	return 0;
}

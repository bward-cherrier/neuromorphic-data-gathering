#include <dv-processing/io/network_reader.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	// Initiate the client connection to the same port and localhost loopback address
	dv::io::NetworkReader client("127.0.0.1", 10101);

	// Validate that this client is connected to an event data stream
	if (!client.isEventStreamAvailable()) {
		throw dv::exceptions::RuntimeError("Server does not provide event data!");
	}

	// Initialize the event visualizer with server reported sensor resolution
	dv::visualization::EventVisualizer visualizer(client.getEventResolution().value());

	// Create a preview window to show the visualized events
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);

	// Declare an event stream slicer to synchronized event data packets
	dv::EventStreamSlicer slicer;

	// Perform visualization every 10 milliseconds, which should match the server publishing frequency
	slicer.doEveryTimeInterval(10ms, [&visualizer](const dv::EventStore &events) {
		// Display preview image
		cv::imshow("Preview", visualizer.generateImage(events));

		// Short sleep, if user clicks escape key (code 27), exit the application
		if (cv::waitKey(2) == 27) {
			exit(0);
		}
	});

	// While client is connected
	while (client.isRunning()) {
		// Read the event data, validate, and feed into the slicer
		if (const auto events = client.getNextEventBatch(); events.has_value()) {
			slicer.accept(*events);
		}
	}

	return EXIT_SUCCESS;
}

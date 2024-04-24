#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/network_writer.hpp>

int main() {
	using namespace std::chrono_literals;

	// Define image space resolution dimensions
	const cv::Size resolution(200, 200);

	// Define output event stream with valid resolution
	const dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "TEST_DATA", resolution);

	// Initiate the server, needs stream definition to initiate
	dv::io::NetworkWriter server("0.0.0.0", 10101, stream);

	// Print the ready state of the server
	std::cout << "Waiting for connections..." << std::endl;

	// Stream interval defines the packet frequency for this sample
	const dv::Duration streamInterval = 10ms;

	// Starting coordinates of the rectangle data that is going to be sent out in this sample
	cv::Point2i offset(0, 0);

	// Rectangle size in pixels
	const cv::Point2i rectSize(20, 20);

	// A boolean variable used to define movement direction of the rectangle
	bool direction = true;

	// Run indefinitely
	while (true) {
		// Do not produce output if there are no connected clients
		if (server.getClientCount() > 0) {
			// Generate the rectangle at given offset position
			const dv::EventStore events = dv::data::generate::eventRectangle(dv::now(), offset, offset + rectSize);

			// Increase or decrease the position coordinates depending on the "direction" boolean
			offset = direction ? offset + cv::Point2i(1, 1) : offset - cv::Point2i(1, 1);

			// Check if the rectangle coordinates reaches borders of the image
			if (offset.x == 0 || offset.y == 0 || offset.x + rectSize.x == resolution.width
				|| offset.y + rectSize.y == resolution.height) {
				// Reverse the motion direction
				direction = !direction;
			}

			// Send it out to clients
			server.writeEvents(events);
		}

		// Sleep the application for the streaming interval duration
		std::this_thread::sleep_for(streamInterval);
	}

	return EXIT_SUCCESS;
}

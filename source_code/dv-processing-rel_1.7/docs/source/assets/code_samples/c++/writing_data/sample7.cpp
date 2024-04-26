#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Create a stream named "keypoints" for a data type of TimedKeyPoint packet
	auto config = dv::io::MonoCameraWriter::Config("DVXplorer_sample");

	// Define and contain a stream name in a variable
	const std::string streamName = "keypoints";

	// Add an output stream with a packet type and a defined stream name
	config.addStream<dv::TimedKeyPointPacket>(streamName);

	// Initialize the writer
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Let's create 10 packets of key-points
	for (int i = 0; i < 10; i++) {
		// Create a packet for writing
		dv::TimedKeyPointPacket packet;

		// Generate some monotonically increasing timestamp
		const int64_t timestamp = i * 1000;

		// 10 Let's generate 10 key-points for the packet
		for (int y = 0; y < 10; y++) {
			// Using emplace_back to directly allocate and insert the keypoint at the end of the vector
			packet.elements.emplace_back(dv::Point2f(5.f, 5.f), 10.f, 3.f, 1.f, 0, -1, timestamp);
		}

		// Write the packet using, requires a stream name that is the same with the
		writer.writePacket(packet, streamName);
	}

	return 0;
}

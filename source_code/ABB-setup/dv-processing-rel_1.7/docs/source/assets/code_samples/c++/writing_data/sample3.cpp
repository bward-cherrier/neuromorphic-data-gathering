#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Sample VGA resolution, same as the DVXplorer camera
	const cv::Size resolution(640, 480);

	// Event only configuration
	const auto config = dv::io::MonoCameraWriter::EventOnlyConfig("DVXplorer_sample", resolution);

	// Create the writer instance, it will only have a single event output stream.
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Write 100 packet of event data
	for (int i = 0; i < 100; i++) {
		// EventStore requires strictly monotonically increasing data, generate
		// a timestamp from the iteration counter value
		const int64_t timestamp = i * 10000;

		// Generate sample event batch
		dv::EventStore events = dv::data::generate::dvLogoAsEvents(timestamp, resolution);

		// Write the packet using the writer, the data is not going be written at the exact
		// time of the call to this function, it is only guaranteed to be written after
		// the writer instance is destroyed (destructor has completed)
		writer.writeEvents(events);
	}

	return 0;
}

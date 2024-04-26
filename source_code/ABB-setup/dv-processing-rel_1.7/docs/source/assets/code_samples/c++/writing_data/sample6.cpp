#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// Trigger data only configuration
	auto config = dv::io::MonoCameraWriter::Config("DVXplorer_sample");
	config.addTriggerStream();

	// Create the writer instance, it will only have a single trigger output stream
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Write 100 triggers
	for (int i = 0; i < 100; i++) {
		// Generate some monotonically increasing timestamp
		const int64_t timestamp = i * 1000;

		// Single trigger instance, let's say this is some signal from external source
		dv::Trigger trigger(timestamp, dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE);

		// Write the trigger value
		writer.writeTrigger(trigger);
	}

	return 0;
}

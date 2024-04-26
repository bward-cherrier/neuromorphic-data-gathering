#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

int main() {
	// IMU data only configuration
	auto config = dv::io::MonoCameraWriter::Config("DVXplorer_sample");
	config.addImuStream();

	// Create the writer instance, it will only have a single IMU data output stream
	dv::io::MonoCameraWriter writer("mono_writer_sample.aedat4", config);

	// Write 100 IMU measurements
	for (int i = 0; i < 100; i++) {
		// Generate some monotonically increasing timestamp
		const int64_t timestamp = i * 1000;

		// Some sample measurements - no rotation on gyro and a one-G gravity on Y axis of accelerometer
		const dv::IMU measurement = dv::data::generate::levelImuWithNoise(timestamp);

		// Write the measurement
		writer.writeImu(measurement);
	}

	return 0;
}

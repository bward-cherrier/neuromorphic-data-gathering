import dv_processing as dv

# IMU data only configuration
config = dv.io.MonoCameraWriter.Config("DVXplorer_sample")
config.addImuStream()

# Create the writer instance, it will only have a single IMU data output stream
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

# Write 100 IMU measurements
for i in range(100):
    # Generate some monotonically increasing timestamp
    timestamp = i * 1000

    # Single IMU measurement instance
    measurement = dv.data.generate.levelImuWithNoise(timestamp)

    # Write the measurement
    writer.writeImu(measurement)

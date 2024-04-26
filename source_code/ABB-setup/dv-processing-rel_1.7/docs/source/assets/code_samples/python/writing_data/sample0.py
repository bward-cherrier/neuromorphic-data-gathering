import dv_processing as dv

config = dv.io.MonoCameraWriter.Config("DVXplorer_sample")

# Define VGA resolution for this camera
resolution = (640, 480)

# Add an event stream with a resolution
config.addEventStream(resolution)

# Add frame stream with a resolution
config.addFrameStream(resolution)

# Add IMU stream
config.addImuStream()

# Add trigger stream
config.addTriggerStream()

# Create the writer instance with the configuration structure
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

print(f"Is event stream available? {str(writer.isEventStreamConfigured())}")
print(f"Is frame stream available? {str(writer.isFrameStreamConfigured())}")
print(f"Is imu stream available? {str(writer.isImuStreamConfigured())}")
print(f"Is trigger stream available? {str(writer.isTriggerStreamConfigured())}")

import dv_processing as dv

# Create a DVS config - events, imu, and triggers are going to be enabled
config = dv.io.MonoCameraWriter.DVSConfig("DVXplorer_sample", (640, 480))

# Create the writer instance with the configuration structure
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

# Print which streams were configured
print(f"Is event stream available? {str(writer.isEventStreamConfigured())}")
print(f"Is frame stream available? {str(writer.isFrameStreamConfigured())}")
print(f"Is imu stream available? {str(writer.isImuStreamConfigured())}")
print(f"Is trigger stream available? {str(writer.isTriggerStreamConfigured())}")

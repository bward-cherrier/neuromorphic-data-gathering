import dv_processing as dv

capture = dv.io.CameraCapture()

writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", capture)

print(f"Is event stream available? {str(writer.isEventStreamConfigured())}")
print(f"Is frame stream available? {str(writer.isFrameStreamConfigured())}")
print(f"Is imu stream available? {str(writer.isImuStreamConfigured())}")
print(f"Is trigger stream available? {str(writer.isTriggerStreamConfigured())}")

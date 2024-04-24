import time
import dv_processing as dv

# Open any camera
capture = dv.io.CameraCapture()

# Run the loop while camera is still connected
while capture.isRunning():
    # Read a batch of IMU data from the camera
    imu_batch = capture.getNextImuBatch()

    # The method does not wait for data to arrive, it returns immediately with
    # latest available data or if no data is available, returns a `None`
    if imu_batch is not None and len(imu_batch) > 0:
        # Print the time range of imu data
        print(f"Received imu data within time range [{imu_batch[0].timestamp}; {imu_batch[-1].timestamp}]")
    else:
        time.sleep(0.001)

import dv_processing as dv

# Open a file
reader = dv.io.MonoCameraRecording("path/to/file.aedat4")

# Run the loop while stream contains data
while reader.isRunning():
    # Read a batch of IMU data from the camera
    imu_batch = reader.getNextImuBatch()
    if imu_batch is not None and len(imu_batch) > 0:
        # Print the info of the imu data
        print(f"Received {len(imu_batch)} IMU measurements")

import dv_processing as dv

# Store the file path
path_to_file = "path/to/file.aedat4"

# Open a file
reader = dv.io.MonoCameraRecording(path_to_file)

# Print file path and camera name
print(f"Checking available streams in [{path_to_file}] for camera name [{reader.getCameraName()}]:")

# Check if event stream is available
if reader.isEventStreamAvailable():
    # Check the resolution of event stream
    resolution = reader.getEventResolution()

    # Print that the stream is present and its resolution
    print(f"  * Event stream with resolution [{resolution.width}x{resolution.height}]")

# Check if frame stream is available
if reader.isFrameStreamAvailable():
    # Check the resolution of frame stream
    resolution = reader.getFrameResolution()

    # Print that the stream is available and its resolution
    print(f"  * Frame stream with resolution [{resolution.width}x{resolution.height}]")

# Check if IMU stream is available
if reader.isImuStreamAvailable():
    # Print that the IMU stream is available
    print("  * IMU stream")

# Check if trigger stream is available
if reader.isTriggerStreamAvailable():
    # Print that the trigger stream is available
    print("  * Trigger stream")

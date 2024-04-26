import dv_processing as dv

# Open a file
reader = dv.io.MonoCameraRecording("path/to/file.aedat4")

# Run the loop while camera is still connected
while reader.isRunning():
    # Read a a batch of triggers from the camera
    triggers = reader.getNextTriggerBatch()

    # Check whether batch is valid and contains data
    if triggers is not None and len(triggers) > 0:
        # Print the trigger batch information
        print(f"Received {len(triggers)} triggers")

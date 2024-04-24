import dv_processing as dv

# Open any camera
reader = dv.io.MonoCameraRecording("path/to/file.aedat4")

# Run the loop while camera is still connected
while reader.isRunning():
    # Read batch of events
    events = reader.getNextEventBatch()
    if events is not None:
        # Print received packet time range
        print(f"{events}")

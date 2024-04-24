import dv_processing as dv
import time

# Open any camera
capture = dv.io.CameraCapture()

# Run the loop while camera is still connected
while capture.isRunning():
    # Read batch of events
    events = capture.getNextEventBatch()

    # The method does not wait for data arrive, it returns immediately with
    # latest available data or if no data is available, returns a `None`
    if events is not None:
        # Print received packet time range
        print(f"Received events within time range [{events.getLowestTime()}; {events.getHighestTime()}]")
    else:
        # No data has arrived yet, short sleep to reduce CPU load
        time.sleep(0.001)

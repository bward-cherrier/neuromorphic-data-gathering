import time
import dv_processing as dv

# Open any camera
capture = dv.io.CameraCapture()

# Depending on the incoming signal, enable the detection of the desired type of pattern, here we enable everything.
# Enable rising edge detection
capture.deviceConfigSet(4, 1, True)
# Enable falling edge detection
capture.deviceConfigSet(4, 2, True)
# Enable pulse detection
capture.deviceConfigSet(4, 3, True)
# Enable detector
capture.deviceConfigSet(4, 0, True)

# Run the loop while camera is still connected
while capture.isRunning():
    # Read a batch of triggers from the camera
    triggers = capture.getNextTriggerBatch()

    # The method does not wait for data arrive, it returns immediately with
    # latest available data or if no data is available, returns a `None`
    if triggers is not None and len(triggers) > 0:
        # Print the time range of trigger data
        print(f"Received trigger data within time range [{triggers[0].timestamp}; {triggers[-1].timestamp}]")
    else:
        time.sleep(0.001)

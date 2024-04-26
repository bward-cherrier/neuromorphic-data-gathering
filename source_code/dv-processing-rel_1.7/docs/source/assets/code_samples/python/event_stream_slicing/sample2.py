import dv_processing as dv
from datetime import timedelta

# Discover connected camera to the system
cameras = dv.io.discoverDevices()
if len(cameras) < 2:
    raise RuntimeError("Unable to discover two cameras")

# Open the cameras, just use first detected cameras
stereo = dv.io.StereoCapture(cameras[0], cameras[1])

# Initialize a stereo stream slicer
slicer = dv.io.StereoEventStreamSlicer()


# Callback method for time based slicing
def print_time_interval(left_events: dv.EventStore, right_events: dv.EventStore):
    # Print the time duration received by this method
    print(f"* Received events with duration: left[{left_events.duration()}] - right[{right_events.duration()}]")


# Register a job to be performed every 33 milliseconds
slicer.doEveryTimeInterval(timedelta(milliseconds=33), print_time_interval)


# Callback method for number based slicing
def print_event_number(left_events: dv.EventStore, right_events: dv.EventStore):
    # Print the number of events received here
    print(
        f"# Received events in number-based slicing, counts: left[{left_events.size()}] - right[{right_events.size()}]")


# Register this method to be called every 1000 events
slicer.doEveryNumberOfEvents(1000, print_event_number)

# Continue the loop while both cameras are connected
while stereo.left.isRunning() and stereo.right.isRunning():
    # Initialize empty stores
    left = dv.EventStore()
    right = dv.EventStore()

    # Receive packet from left camera
    leftPacket = stereo.left.getNextEventBatch()
    # Assign the packet if some data was received
    if leftPacket is not None:
        left = leftPacket

    # Receive packet from right camera
    rightPacket = stereo.right.getNextEventBatch()
    # Assign the packet if some data was received
    if rightPacket is not None:
        right = rightPacket

    # Pass all events into the slicer
    slicer.accept(left, right)

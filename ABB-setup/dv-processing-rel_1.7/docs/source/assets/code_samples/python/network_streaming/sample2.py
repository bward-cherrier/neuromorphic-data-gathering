import dv_processing as dv
import cv2 as cv
from datetime import timedelta

# Initiate the client connection to the same port and localhost loopback address
client = dv.io.NetworkReader("127.0.0.1", 10101)

# Validate that this client is connected to an event data stream
if not client.isEventStreamAvailable():
    raise RuntimeError("Server does not provide event data!")

# Initialize the event visualizer with server reported sensor resolution
visualizer = dv.visualization.EventVisualizer(client.getEventResolution())

# Create a preview window to show the visualized events
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Declare an event stream slicer to synchronized event data packets
slicer = dv.EventStreamSlicer()


# Callback method to show the generated event visualization
def show_preview(events: dv.EventStore):
    # Display preview image
    cv.imshow("Preview", visualizer.generateImage(events))

    # Short sleep, if user clicks escape key (code 27), exit the application
    if cv.waitKey(2) == 27:
        exit(0)


# Perform visualization every 10 milliseconds, which should match the server publishing frequency
slicer.doEveryTimeInterval(timedelta(milliseconds=10), show_preview)

# While client is connected
while True:
    # Read the event data
    events = client.getNextEventBatch()

    # Validate the data and feed into the slicer
    if events is not None:
        slicer.accept(events)

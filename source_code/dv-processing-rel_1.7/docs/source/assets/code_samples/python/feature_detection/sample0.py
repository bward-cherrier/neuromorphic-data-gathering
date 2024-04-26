import dv_processing as dv
import cv2 as cv
from datetime import timedelta

# Open any camera
capture = dv.io.CameraCapture()

# Make sure it supports event stream output, throw an error otherwise
if not capture.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide an event stream.")

# Initialize an accumulator with some resolution
accumulator = dv.Accumulator(capture.getEventResolution())

# Initialize preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Let's detect 100 features
number_of_features = 100

# Create an image feature detector with given resolution. By default, it uses FAST feature detector
# with AdaptiveNMS post-processing.
detector = dv.features.ImageFeatureDetector(capture.getEventResolution())

# Initialize a slicer
slicer = dv.EventStreamSlicer()


# Declare the callback method for slicer
def slicing_callback(events: dv.EventStore):
    # Pass events into the accumulator and generate a preview frame
    accumulator.accept(events)
    frame = accumulator.generateFrame()

    # Run the feature detection on the accumulated frame
    features = detector.runDetection(frame, number_of_features)

    # Create a colored preview image by converting from grayscale to BGR
    preview = cv.cvtColor(frame.image, cv.COLOR_GRAY2BGR)
    for feature in features:
        # Draw a rectangle marker on each feature location
        cv.drawMarker(preview, (int(feature.pt[0]), int(feature.pt[1])),
                      dv.visualization.colors.someNeonColor(feature.class_id), cv.MARKER_SQUARE, 10, 2)

    # Show the accumulated image
    cv.imshow("Preview", preview)
    cv.waitKey(2)


# Register a callback every 33 milliseconds
slicer.doEveryTimeInterval(timedelta(milliseconds=33), slicing_callback)

# Run the event processing while the camera is connected
while capture.isRunning():
    # Receive events
    events = capture.getNextEventBatch()

    # Check if anything was received
    if events is not None:
        # If so, pass the events into the slicer to handle them
        slicer.accept(events)

import dv_processing as dv
import cv2 as cv
from datetime import timedelta


def preprocess(image):
    # add your pre precessing here..
    return image


# Open any camera
capture = dv.io.CameraCapture()

# Make sure it supports event stream output, throw an error otherwise
if not capture.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide an event stream.")

# Initialize an accumulator with some resolution (for visualization only)
accumulator = dv.Accumulator(capture.getEventResolution())

# Initialize preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Let's detect 100 features
number_of_features = 100

# Define number of pyr-down to be applied to the image before detecting blobs. If pyramidLevel is set to zero, the
# blobs will be detected on the original image resolution, if pyramid level is one, the image will be down sampled by a
# factor of 2 (factor of 4 if pyramid level is 2 and so on..) before perfirming the detection. The blobs will then be
# scaled back to the original resolution size.
pyramidLevel = 0

# Create an event-based blob detector
detector = dv.features.EventFeatureBlobDetector(capture.getEventResolution(), pyramidLevel, preprocess)

# Initialize a slicer
slicer = dv.EventStreamSlicer()


# Declare the callback method for slicer
def slicing_callback(events: dv.EventStore):
    # Run the feature detection on the incoming events
    features = detector.runDetection(events, number_of_features)

    # Pass events into the accumulator and generate a preview frame
    accumulator.accept(events)
    frame = accumulator.generateFrame()

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

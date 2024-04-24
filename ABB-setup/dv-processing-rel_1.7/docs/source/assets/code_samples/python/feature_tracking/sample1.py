import dv_processing as dv
import cv2 as cv

# Open any camera
capture = dv.io.CameraCapture()

# Make sure it supports event stream output, throw an error otherwise
if not capture.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide an event stream.")

# Initialize preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Instantiate a visual tracker with known resolution, all parameters kept default
tracker = dv.features.EventFeatureLKTracker.RegularTracker(capture.getEventResolution())

# Run tracking by accumulating frames with 100 FPS
tracker.setFramerate(100)

# Create a track container instance that is used to visualize tracks on an image
tracks = dv.features.FeatureTracks()

# Run the frame processing while the camera is connected
while capture.isRunning():
    # Try to receive a batch of events
    events = capture.getNextEventBatch()

    # Check if anything was received
    if events is not None:
        # Pass the events to the tracker
        tracker.accept(events)

        # Run tracking
        result = tracker.runTracking()

        # Since we are passing events in fine-grained batches, tracking will not execute
        # until enough events is received, returning a `None` if tracking did not execute
        if result is None:
            continue

        # Pass tracking result into the track container which aggregates track history
        tracks.accept(result)

        # Generate and show a preview of recent tracking history
        cv.imshow("Preview", tracks.visualize(tracker.getAccumulatedFrame()))

    cv.waitKey(2)

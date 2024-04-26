import dv_processing as dv
import cv2 as cv
from datetime import timedelta

# Open any camera
capture = dv.io.CameraCapture()

# Make sure it supports correct stream outputs, throw an error otherwise
if not capture.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide an event stream.")
if not capture.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide a frame stream.")

# Initialize preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Instantiate a visual tracker with known resolution, all parameters kept default
tracker = dv.features.EventCombinedLKTracker.RegularTracker(capture.getEventResolution())

# Accumulate and track on 5 intermediate accumulated frames between each actual frame pair
tracker.setNumIntermediateFrames(5)

# Create a track container instance that is used to visualize tracks on an image
tracks = dv.features.FeatureTracks()

# Use a list to store incoming frames to make sure the all data has arrived prior to running the tracking
frame_queue = []

# Run the frame processing while the camera is connected
while capture.isRunning():
    # Try to receive a frame
    frame = capture.getNextFrame()

    # Check if anything was received
    if frame is not None:
        frame_queue.append(frame)

    # Try to receive a batch of events
    events = capture.getNextEventBatch()

    # Check if anything was received
    if events is not None:
        # Pass the events to the tracker
        tracker.accept(events)

        # Check if we have ready frames and if enough events have arrived already
        if len(frame_queue) == 0 or frame_queue[0].timestamp > events.getHighestTime():
            continue

        # Take the last frame from the queue and remove it
        frame = frame_queue.pop(0)

        # Pass it to the tracker as well
        tracker.accept(frame)

        # Run tracking
        result = tracker.runTracking()

        # Validate that the tracking was successful
        if result is None:
            continue

        # Pass tracking result into the track container which aggregates track history
        tracks.accept(result)

        # Generate and show a preview of recent tracking history on both accumulated frames and the frame image
        # Take the set of intermediate accumulated frames from the tracker
        accumulated_frames = tracker.getAccumulatedFrames()
        if len(accumulated_frames) > 0:
            # Draw visualization on both image and concatenate them horizontally
            preview = cv.hconcat([tracks.visualize(accumulated_frames[-1].pyramid[0]), tracks.visualize(frame.image)])

            # Show the final preview image
            cv.imshow("Preview", preview)

    cv.waitKey(2)

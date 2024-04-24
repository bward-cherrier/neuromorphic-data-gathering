import dv_processing as dv
import cv2 as cv

# Open any camera
capture = dv.io.CameraCapture()

# Make sure it supports event stream output, throw an error otherwise
if not capture.isFrameStreamAvailable():
    raise RuntimeError("Input camera does not provide a frame stream.")

# Initialize preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Instantiate a visual tracker with known resolution, all parameters kept default
tracker = dv.features.ImageFeatureLKTracker.RegularTracker(capture.getEventResolution())

# Create a track container instance that is used to visualize tracks on an image
tracks = dv.features.FeatureTracks()

# Run the frame processing while the camera is connected
while capture.isRunning():
    # Try to receive a frame
    frame = capture.getNextFrame()

    # Check if anything was received
    if frame is not None:
        # Pass the frame to the tracker
        tracker.accept(frame)

        # Run tracking
        result = tracker.runTracking()

        # Pass tracking result into the track container which aggregates track history
        tracks.accept(result)

        # Generate and show a preview of recent tracking history
        cv.imshow("Preview", tracks.visualize(frame.image))

    cv.waitKey(2)

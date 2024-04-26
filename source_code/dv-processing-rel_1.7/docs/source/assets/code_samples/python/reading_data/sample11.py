import dv_processing as dv
import cv2 as cv

# Open a file
reader = dv.io.MonoCameraRecording("path/to/file.aedat4")

# Initiate a preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Variable to store the previous frame timestamp for correct playback
lastTimestamp = None

# Run the loop while camera is still connected
while reader.isRunning():
    # Read a frame from the camera
    frame = reader.getNextFrame()

    if frame is not None:
        # Print the timestamp of the received frame
        print(f"Received a frame at time [{frame.timestamp}]")

        # Show a preview of the image
        cv.imshow("Preview", frame.image)

        # Calculate the delay between last and current frame, divide by 1000 to convert microseconds
        # to milliseconds
        delay = (2 if lastTimestamp is None else (frame.timestamp - lastTimestamp) / 1000)

        # Perform the sleep
        cv.waitKey(delay)

        # Store timestamp for the next frame
        lastTimestamp = frame.timestamp

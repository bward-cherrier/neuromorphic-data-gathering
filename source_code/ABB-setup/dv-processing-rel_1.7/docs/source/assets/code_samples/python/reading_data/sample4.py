import dv_processing as dv
import cv2 as cv

# Open any camera
capture = dv.io.CameraCapture()

# Initiate a preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Run the loop while camera is still connected
while capture.isRunning():
    # Read a frame from the camera
    frame = capture.getNextFrame()

    # The method does not wait for frame arrive, it returns immediately with
    # latest available frame or if no data is available, returns a `None`
    if frame is not None:
        # Print received packet time range
        print(f"Received a frame at time [{frame.timestamp}]")

        # Show a preview of the image
        cv.imshow("Preview", frame.image)
    cv.waitKey(2)

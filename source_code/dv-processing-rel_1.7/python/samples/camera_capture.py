import datetime

import dv_processing as dv
import cv2 as cv
import argparse

parser = argparse.ArgumentParser(description='Show a preview of an iniVation event camera input.')

args = parser.parse_args()

# Open the camera
camera = dv.io.CameraCapture()

# Initialize visualizer instance which generates event data preview
visualizer = dv.visualization.EventVisualizer(camera.getEventResolution())

# Create the preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)


def preview_events(event_slice):
    cv.imshow("Preview", visualizer.generateImage(event_slice))
    cv.waitKey(2)


# Create an event slicer, this will only be used events only camera
slicer = dv.EventStreamSlicer()
slicer.doEveryTimeInterval(datetime.timedelta(milliseconds=33), preview_events)

# start read loop
while True:
    # Get events
    events = camera.getNextEventBatch()

    # If no events arrived yet, continue reading
    if events is not None:
        slicer.accept(events)

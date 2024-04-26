import datetime
import dv_processing as dv
import cv2 as cv
import argparse

parser = argparse.ArgumentParser(
    description='Show a preview of a stereo event stream from a pair of time-synchronized iniVation cameras.')

args = parser.parse_args()

# Open the cameras
camera = dv.io.StereoCapture("DVXplorer_DXA00093", "DAVIS346_00000499")

leftVis = dv.visualization.EventVisualizer(camera.left.getEventResolution())
rightVis = dv.visualization.EventVisualizer(camera.right.getEventResolution())

# Create the preview windows
cv.namedWindow("Left", cv.WINDOW_NORMAL)
cv.namedWindow("Right", cv.WINDOW_NORMAL)

slicer = dv.StereoEventStreamSlicer()

keepRunning = True


def preview(left, right):
    cv.imshow("Left", leftVis.generateImage(left))
    cv.imshow("Right", rightVis.generateImage(right))
    if cv.waitKey(2) == 27:
        global keepRunning
        keepRunning = False


slicer.doEveryTimeInterval(datetime.timedelta(milliseconds=33), preview)

while keepRunning:
    # if reading fails, just pass an empty event store
    slicer.accept(camera.left.getNextEventBatch() or dv.EventStore(),
                  camera.right.getNextEventBatch() or dv.EventStore())

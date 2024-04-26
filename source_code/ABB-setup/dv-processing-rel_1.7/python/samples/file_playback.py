import dv_processing as dv
import cv2 as cv
import argparse

parser = argparse.ArgumentParser(description='Show a preview of an AEDAT4 recording.')

parser.add_argument('-f,--file',
                    dest='file',
                    type=str,
                    required=True,
                    metavar='path/to/file',
                    help='Path to an AEDAT4 file')

args = parser.parse_args()

# Open the recording file
recording = dv.io.MonoCameraRecording(args.file)

# Make sure the streams are available
assert recording.isEventStreamAvailable()
assert recording.isFrameStreamAvailable()
assert recording.getFrameResolution() == recording.getEventResolution()

# Initialize event accumulator with the known resolution
acc = dv.Accumulator(recording.getEventResolution())

# Some accumulation parameters
acc.setMaxPotential(1.0)
acc.setEventContribution(0.12)

# Create the preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Last frame is None for the first iteration
lastFrame = None

# Read the first frame from the file
frame = recording.getNextFrame()

# While not end-of-file, None is used to determine the last recorded frame
while frame is not None:
    # We have more than one frame
    if lastFrame is not None:
        # Time delay for the image to be displayed
        delay = frame.timestamp - lastFrame.timestamp

        # Read intermediate events that are available between the last and current frame
        events = recording.getEventsTimeRange(lastFrame.timestamp, frame.timestamp)
        if events is not None:
            # Accumulate the events
            acc.accept(events)

        # Retrieve accumulated image
        accumulatedFrame = acc.generateFrame()

        # If frames have more than 2 dimensions, convert the accumulated image into BGR colorspace
        if len(lastFrame.image.shape) > 2:
            accumulatedFrame.image = cv.cvtColor(accumulatedFrame.image, cv.COLOR_GRAY2BGR)

        # Concat the images and preview them
        cv.imshow("Preview", cv.hconcat([lastFrame.image, accumulatedFrame.image]))
        cv.waitKey(int(delay / 1000))

    # Cache the last read frame and move to the next frame
    lastFrame = frame
    frame = recording.getNextFrame()

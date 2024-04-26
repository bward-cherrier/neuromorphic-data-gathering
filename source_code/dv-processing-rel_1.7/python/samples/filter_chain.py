import datetime

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

# Make sure we have event stream available
assert recording.isEventStreamAvailable()

# Create an accumulator instance for preview generation
acc = dv.EdgeMapAccumulator(recording.getEventResolution())

filter_chain = dv.EventFilterChain()
# Filter refractory period
filter_chain.addFilter(dv.RefractoryPeriodFilter(recording.getEventResolution()))
# Only positive events
filter_chain.addFilter(dv.EventPolarityFilter(True))
# Remove noise
filter_chain.addFilter(dv.noise.BackgroundActivityNoiseFilter(recording.getEventResolution()))

# Create the preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)


# Apply filter chain and show preview
def filter_events(events):
    # Pass data to filter
    filter_chain.accept(events)
    # apply filtering
    filtered_events = filter_chain.generateEvents()
    # pass the data into the accumulator
    acc.accept(filtered_events)
    # Generate frame and show preview
    frame = acc.generateFrame()
    cv.imshow("Preview", frame.image)
    cv.waitKey(2)


# Initiate the event stream slicer
slicer = dv.EventStreamSlicer()
slicer.doEveryTimeInterval(datetime.timedelta(milliseconds=33), filter_events)

# Read the first frame from the file
events = recording.getNextEventBatch()

# While not end-of-file, None is used to determine the last recorded frame
while events is not None:
    slicer.accept(events)
    events = recording.getNextEventBatch()

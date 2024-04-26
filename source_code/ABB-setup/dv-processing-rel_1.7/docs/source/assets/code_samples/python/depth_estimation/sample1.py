import dv_processing as dv
import cv2 as cv
from datetime import timedelta

# Path to a stereo calibration file, replace with a file path on your local file system
calibration_file_path = "path/to/calibration.json"

# Load the calibration file
calibration = dv.camera.CalibrationSet.LoadFromFile(calibration_file_path)

# It is expected that calibration file will have "C0" as the leftEventBuffer camera
left_camera = calibration.getCameraCalibration("C0")

# The second camera is assumed to be rightEventBuffer-side camera
right_camera = calibration.getCameraCalibration("C1")

# Open the stereo camera with camera names from calibration
capture = dv.io.StereoCapture(left_camera.name, right_camera.name)

# Make sure both cameras support event stream output, throw an error otherwise
if not capture.left.isEventStreamAvailable() or not capture.right.isEventStreamAvailable():
    raise RuntimeError("Input camera does not provide an event stream.")

# Matching window size for the block matcher
window = (24, 24)

# Minimum disparity value to measure
min_disparity = 0

# Maximum disparity value
max_disparity = 40

# Minimum z-score value that a valid match can have
min_score = 0.0

# Initialize the block matcher with rectification
matcher = dv.SparseEventBlockMatcher(dv.camera.StereoGeometry(left_camera, right_camera), window, max_disparity,
                                     min_disparity, min_score)

# Initialization of a stereo event sliver
slicer = dv.StereoEventStreamSlicer()

# Initialize a window to show previews of the output
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Local event buffers to implement overlapping window of events for accumulation
global left_event_buffer, right_event_buffer
left_event_buffer = dv.EventStore()
right_event_buffer = dv.EventStore()

# Use one third of the resolution as count of events per accumulated frame
event_count = int((left_camera.resolution[0] * left_camera.resolution[1]) / 3)


# Stereo slicer callback method
def callback(left_events: dv.EventStore, right_events: dv.EventStore):
    # Push input events into the local buffers
    global left_event_buffer, right_event_buffer
    left_event_buffer.add(left_events)
    right_event_buffer.add(right_events)

    # If the number of events is above the count, just keep the latest events
    if len(left_event_buffer) > event_count:
        left_event_buffer = left_event_buffer.sliceBack(event_count)
    if len(right_event_buffer) > event_count:
        right_event_buffer = right_event_buffer.sliceBack(event_count)

    # Number of clusters to extract
    num_clusters = 100

    # Initialize the mean-shift clustering algorithm
    mean_shift = dv.cluster.mean_shift.MeanShiftEventStoreAdaptor(left_event_buffer, 10, 1, 20, num_clusters)

    # Find cluster centers which are going to be used for disparity estimation
    centers = mean_shift.findClusterCentresEpanechnikov()

    # Run disparity estimation, the output will contain a disparity estimate for each of the given points.
    estimates = matcher.computeDisparitySparse(left_event_buffer, right_event_buffer, list(map(lambda x: x.pt,
                                                                                               centers)))

    # Convert the accumulated frames into colored images for preview.
    images = []
    images.append(cv.cvtColor(matcher.getLeftFrame().image, cv.COLOR_GRAY2BGR))
    images.append(cv.cvtColor(matcher.getRightFrame().image, cv.COLOR_GRAY2BGR))

    # Visualize the matched blocks
    index = 0
    for point in estimates:
        # If point estimation is invalid, do not show a preview of it
        if not point.valid:
            continue

        # The rest of the code performs drawing of the match according to the disparity value on the
        # preview images.
        color = dv.visualization.colors.someNeonColor(index)
        index += 1

        # Draw some nice colored markers and rectangles.
        cv.drawMarker(images[1], point.matchedPosition, color, cv.MARKER_CROSS, 7)
        cv.rectangle(images[1],
                     (int(point.matchedPosition[0] - (window[0] / 2)), int(point.matchedPosition[1] - (window[1] / 2))),
                     (int(point.matchedPosition[0] + (window[0] / 2)), int(point.matchedPosition[1] + (window[1] / 2))),
                     color)
        cv.rectangle(
            images[0],
            (int(point.templatePosition[0] - (window[0] / 2)), int(point.templatePosition[1] - (window[1] / 2))),
            (int(point.templatePosition[0] + (window[0] / 2)), int(point.templatePosition[1] + (window[1] / 2))), color)

    # Concatenate images and show them in a window
    cv.imshow("Preview", cv.hconcat(images))


# Register a callback to be done at 30Hz
slicer.doEveryTimeInterval(timedelta(milliseconds=33), callback)

# Buffer input events in these variables to synchronize inputs
left_events = None
right_events = None

# Run the processing loop while both cameras are connected
while capture.left.isRunning() and capture.right.isRunning():
    # Read events from respective left / right cameras
    if left_events is None:
        left_events = capture.left.getNextEventBatch()
    if right_events is None:
        right_events = capture.right.getNextEventBatch()

    # Feed the data into the slicer and reset the buffer
    if left_events is not None and right_events is not None:
        slicer.accept(left_events, right_events)
        left_events = None
        right_events = None

    #  Wait for a small amount of time to avoid CPU overhaul
    cv.waitKey(1)

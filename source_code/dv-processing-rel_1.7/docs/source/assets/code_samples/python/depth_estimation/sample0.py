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

# Initialize a stereo block matcher with a stereo geometry from calibration and the preconfigured SGBM instance
block_matcher = dv.SemiDenseStereoMatcher(dv.camera.StereoGeometry(left_camera, right_camera))

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

    # Pass these events into block matcher and estimate disparity, the matcher will accumulate frames
    # internally. The disparity output is 16-bit integer, that has sub-pixel precision.
    disparity = block_matcher.computeDisparity(left_event_buffer, right_event_buffer)

    # Convert the accumulated frames into colored images for preview.
    images = []
    images.append(cv.cvtColor(block_matcher.getLeftFrame().image, cv.COLOR_GRAY2BGR))
    images.append(cv.cvtColor(block_matcher.getRightFrame().image, cv.COLOR_GRAY2BGR))

    # Convert disparity into 8-bit integers with scaling and normalize the output for a nice preview.
    # This loses the actual numeric value of the disparity, but it's a nice way to visualize the disparity.
    # Apply color-mapping to the disparity image, this will encode depth with color: red - close; blue - far.
    images.append(cv.applyColorMap(cv.normalize(disparity, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1), cv.COLORMAP_JET))

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

import dv_processing as dv
import cv2 as cv
from datetime import timedelta

# Hardcoded VGA resolution
resolution = (640, 480)

# Initializing input events with uniformly distributed events which represent noise
events = dv.data.generate.uniformEventsWithinTimeRange(0, timedelta(milliseconds=10), resolution, 1000)

# Adding additional data for drawing, this will give an idea whether the filter removes actual signal events
events.add(dv.data.generate.dvLogoAsEvents(10000, resolution))

# Initialize a background activity noise filter with 10-millisecond half life decay, resolution subdivision
# factor of 4 and noise threshold of 1. Half life decay and noise threshold values controls the quality of
# filtering, while subdivision factor is used for resolution downsizing for internal event representation.
filter = dv.noise.FastDecayNoiseFilter(resolution,
                                       halfLife=timedelta(milliseconds=10),
                                       subdivisionFactor=4,
                                       noiseThreshold=1.0)

# Pass events to the filter
filter.accept(events)

# Call generate events to apply the noise filter
filtered = filter.generateEvents()

# Print out the reduction factor, which indicates the percentage of discarded events
print(f"Filter reduced number of events by a factor of {filter.getReductionFactor()}")

# Use a visualizer instance to preview the events
visualizer = dv.visualization.EventVisualizer(resolution)

# Generate preview images of data input and output
input = visualizer.generateImage(events)
output = visualizer.generateImage(filtered)

# Concatenate the images into a single image for preview
preview = cv.hconcat([input, output])

# Display the input and output images
cv.namedWindow("preview", cv.WINDOW_NORMAL)
cv.imshow("preview", preview)
cv.waitKey()

import dv_processing as dv
import cv2 as cv
from datetime import timedelta

resolution = (200, 200)

# Initializing input events with events that represent a logo
events = dv.data.generate.dvLogoAsEvents(0, resolution)

# Filter positive polarity events only
filter = dv.EventPolarityFilter(True)

# Pass events to the filter
filter.accept(events)

# Call generate events to apply the filter
filtered = filter.generateEvents()

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

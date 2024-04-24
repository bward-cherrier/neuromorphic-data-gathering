import dv_processing as dv
import cv2 as cv
from datetime import timedelta

resolution = (200, 200)

# Initializing input events with events that represent a logo
events = dv.data.generate.dvLogoAsEvents(0, resolution)

# Initialize event filter chain, it contains no filters
filter = dv.EventFilterChain()

# Now let's add filters
# First, add a region filter with hardcoded coordinates
filter.addFilter(dv.EventRegionFilter((50, 50, 100, 100)))

# Second, add a positive polarity filter
filter.addFilter(dv.EventPolarityFilter(True))

# Third, add a background activity noise filter
filter.addFilter(dv.noise.BackgroundActivityNoiseFilter(resolution))

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

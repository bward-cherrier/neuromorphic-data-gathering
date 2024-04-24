import dv_processing as dv
import cv2 as cv
import numpy as np

# Smaller resolution for previews
resolution = (200, 200)

# Initializing input events with events that represent a logo
events = dv.data.generate.dvLogoAsEvents(0, resolution)

# Initialize a mask with all zero values
mask = np.full(resolution, fill_value=0, dtype=np.uint8)

# Draw two rectangles to generate a similar to checkerboard mask pattern
cv.rectangle(mask, [0, 0], [int(resolution[0] / 2), int(resolution[1] / 2)], (255, ), cv.FILLED)
cv.rectangle(mask, [int(resolution[0] / 2), int(resolution[1] / 2)], resolution, (255, ), cv.FILLED)

# Initialize the mask filter with the generated mask
filter = dv.EventMaskFilter(mask)

# Pass events to the filter
filter.accept(events)

# Call generate events to apply the filter
filtered = filter.generateEvents()

# Print out the reduction factor, which indicates the percentage of discarded events
print(f"Filter reduced number of events by a factor of {filter.getReductionFactor()}")

# Use a visualizer instance to preview the events
visualizer = dv.visualization.EventVisualizer(resolution)

# Generate preview images of data input and output
input = visualizer.generateImage(events)
output = visualizer.generateImage(filtered)

# Concatenate the images into a single image for preview
preview = cv.hconcat([input, cv.cvtColor(mask, cv.COLOR_GRAY2BGR), output])

# Display the input and output images
cv.namedWindow("preview", cv.WINDOW_NORMAL)
cv.imshow("preview", preview)
cv.waitKey()

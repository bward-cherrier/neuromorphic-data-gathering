import dv_processing as dv
import cv2 as cv
from datetime import timedelta

resolution = (200, 200)

# Initializing 10000 events that are uniformly spaced in pixel area and time
events = dv.data.generate.uniformEventsWithinTimeRange(0, timedelta(milliseconds=10), resolution, 10000)

# Initialize refractory period filter with 1-millisecond period
filter = dv.RefractoryPeriodFilter(resolution, timedelta(milliseconds=1))

# Pass events to the filter
filter.accept(events)

# Call generate events to apply the filter
filtered = filter.generateEvents()

# Print out the number of events after filtering
print(f"Filtered [{len(filtered)}] events ouf of [{len(events)}]")

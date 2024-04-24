import dv_processing as dv
import cv2 as cv

# Use VGA resolution
resolution = (640, 480)

# Initialize a list of clusters for synthetic data generation
clusters = [(100, 100), (462, 25), (105, 340), (540, 420)]

# Generate some random events for a background
events = dv.data.generate.uniformlyDistributedEvents(0, resolution, 10000)

# Declare a region filter which we will use to filter out-of-bounds events in the next step
filter = dv.EventRegionFilter((0, 0, resolution[0], resolution[1]))

for cluster in clusters:
    # Generate a batch of normally distributed events around each of the cluster centers
    filter.accept(dv.data.generate.normallyDistributedEvents(0, cluster, (15, 15), 5000))

    # Apply region filter to the events to filter out events outside valid dimensions
    events.add(filter.generateEvents())

# Initialize mean shift clustering algorithm, with initial parameters of:
# bandwidth = 100, this is pixel search radius around a point
# conv = 0.01, the search converges when the magnitude of mean-shift vector is below this value
# maxIter = 10000, maximum number of mean-shift update iterations
# numStartingPoints = 100, number of randomly selected starting points
mean_shift = dv.cluster.mean_shift.MeanShiftEventStoreAdaptor(events, 100, 0.01, 10000, 100)

# Perform the mean-shift, the algorithm returns a tuple of center coordinates, labels, count of event in the
# cluster, and variances of the cluster
centers, labels, counts, variances = mean_shift.fit()

# Let's assign the cluster size to the response value of the center keypoint
for i in range(len(centers)):
    centers[i].response = counts[i]

# Sort the estimated centers by the number of events in cluster, the values are sorted in descending order
centers.sort(key=lambda a: a.response, reverse=True)

# Choose top four center with most events; these centers should be close to initial hardcoded cluster centers
if len(centers) > 4:
    centers = centers[:4]

# Use event visualizer to generate a preview image
visualizer = dv.visualization.EventVisualizer(resolution)
preview = visualizer.generateImage(events)

# Draw markers on each of the center coordinates
for center in centers:
    cv.drawMarker(preview, (int(center.pt[0]), int(center.pt[1])), dv.visualization.colors.red())

# Show the preview image with detected cluster centers
cv.namedWindow("Preview", cv.WINDOW_NORMAL)
cv.imshow("Preview", preview)
cv.waitKey()

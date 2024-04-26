import datetime

import dv_processing as dv
import cv2 as cv


def generate_event_clusters_at_time(time, clusters, num_iter, shift=-5):
    # Declare a region filter which we will use to filter out-of-bounds events in the next step
    event_filter = dv.EventRegionFilter((0, 0, resolution[0], resolution[1]))
    event_filtered = dv.EventStore()
    track_id = 0
    offset = shift * num_iter
    for cluster in clusters:
        x_coord = cluster[0] + offset
        y_coord = cluster[1] + offset

        # Generate a batch of normally distributed events around each of the cluster centers
        event_filter.accept(dv.data.generate.normallyDistributedEvents(time, (x_coord, y_coord), (3, 3), 1000))

        # Apply region filter to the events to filter out events outside valid dimensions
        event_filtered.add(event_filter.generateEvents())

        track_id += 1

    return event_filtered


def run_mean_shift(events):
    mean_shift.accept(events)
    mean_shift_tracks = mean_shift.runTracking()

    preview = visualizer.generateImage(events)

    # Draw markers on each of the track coordinates
    if len(mean_shift_tracks.keypoints) > 0:
        for index in range(len(mean_shift_tracks.keypoints)):
            track = mean_shift_tracks.keypoints[index]
            cv.drawMarker(preview, (int(track.pt[0]), int(track.pt[1])), dv.visualization.colors.red(), cv.MARKER_CROSS,
                          20, 2)

    # Show the preview image with detected tracks
    cv.imshow("Preview", preview)
    cv.waitKey(10)


# Use VGA resolution
resolution = (640, 480)

# Initialize a slicer
slicer = dv.EventStreamSlicer()

# Initialize a preview window
cv.namedWindow("Preview", cv.WINDOW_NORMAL)

# Initialize a list of clusters for synthetic data generation
clusters = [(550, 400), (70, 300), (305, 100)]

# Generate some random events for a background
events = dv.data.generate.uniformlyDistributedEvents(0, resolution, 10000)

timestamps = [0, 40000, 80000, 120000, 160000, 200000, 240000, 280000, 320000, 360000]

num_iter = 0
for time in timestamps:
    event_cluster = generate_event_clusters_at_time(time, clusters, num_iter)
    events.add(event_cluster)
    events.add(dv.data.generate.uniformlyDistributedEvents(time, resolution, 10000, num_iter))
    num_iter += 1

# parameter defining the spatial window [pixels] in which the new track position will be searched
bandwidth = 10

# window of time used to compute the time surface used for the tracking update
time_window = datetime.timedelta(milliseconds=50)

# Initialize a mean shift tracker
mean_shift = dv.features.MeanShiftTracker(resolution, bandwidth, time_window)

visualizer = dv.visualization.EventVisualizer(resolution)

slicer.doEveryTimeInterval(datetime.timedelta(milliseconds=40), run_mean_shift)

slicer.accept(events)

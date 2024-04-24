# Feature tracking

The dv-processing library provides a few algorithm implementations to perform visual tracking of detected features.
Feature tracking was intended for use in the frontends of visual odometry pipelines. While tracking on event input is
feasible, the library also provides frame-based and hybrid (which uses both events and frames) trackers that allow to
build visual odometry pipelines that leverage both input modalities.

## Frame-based tracking

Frame based feature tracking is performed by using Lucas-Kanade tracking algorithm. The following sample shows how to
use the available frame based tracker with a stream of incoming frames.

The following code sample shows how to run a feature tracker on frames coming from a live camera.

```{note}
This sample requires a camera that is capable of producing frames, e.g. a DAVIS series camera.
```

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_tracking/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_tracking/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/feature_tracking/image_lk_tracking.png
---
align: center
---
Tracked features on a live frame from a camera.
```

## Event-based tracking

### Event-based Lucas Kanade tracker

Features can be detected and tracked on a stream of events. The {cpp:class}`dv::features::EventFeatureLKTracker` can
perform this, it accumulates a frame from events internally, runs feature detection and performs Lucas-Kanade tracking
on the accumulated frames.

The following sample code shows how to use the event-only Lucas-Kanade tracker on event stream coming from a live
camera.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_tracking/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_tracking/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/feature_tracking/event_lk_tracking.png
---
align: center
---
Tracked features on a stream of events from a camera.
```

### Event-based mean shift tracker

Detect and track features on a stream of events using mean shift algorithm. Although commonly used for clustering, the
{cpp:class}`dv::features::MeanShiftTracker` class provides a tracking implementation on event data based on mean shift
update. The class internally detects interesting features to track from events (by default it uses
{cpp:class}`dv::features::EventBlobDetector`) and tracks them by running a mean shift update on a normalized time
surface of events. The tracking is performed by following the interesting points detected on the time surface. The
algorithm will shift the tracks towards the latest events, since it takes into account the intensity of the time surface
when performing the track location update.

The algorithm can be summarized as follows:

1. Given a set of events, detect interesting blobs using {cpp:class}`dv::features::EventBlobDetector`. (Note, this step
   happens if no track has been initialized or if redetection is enabled)
1. Compute the time surface representation of a given interval duration.
1. Given a set of input track locations, for each non-converged track retrieve the time surface of events within a
   configured window.
1. Calculate the mean of coordinates for the retrieved neighborhood, weighting each coordinate by the time surface
   intensity value.
1. Shift the initial track location by a mode, which is a vector going from the initial point to the mean coordinate
   multiplied by a learning rate factor.
1. If the mode of a vector is lower than a configured threshold, the track is considered to have converged into the new
   position, otherwise repeat from step one.

This algorithm is useful to track event blobs that could be used as point of interest in event processing algorithms.

The following code sample shows the use of our mean-shift tracker implementation to find and track events on sample data
generated synthetically.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/mean_shift_tracker/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/mean_shift_tracker/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/mean_shift_tracker/tracker_preview.png
---
align: center
---
Expected output of the mean-shift-tracker sample usage. Tracking eight blobs marked with red crosses.
```

## Hybrid tracking

The high-framerate tracking on event stream suggests that the feature tracking on frames can be improved by tracking
features between frames on intermediate accumulated frames from events. The intermediate tracking results can be used as
a prior to the frame tracking algorithm. Such an approach is implemented in
{cpp:class}`dv::features::EventCombinedLKTracker`, it performs regular Lucas-Kanade tracking on frames, but also
constructs intermediate accumulated frames to predict the locations of tracks in the next frame and uses this
information as a prior to the Lucas-Kanade tracking algorithm.

The following sample code shows how to use the hybrid event-frame Lucas-Kanade tracker on both streams coming from a
live camera.

```{note}
This sample requires a camera that is capable of producing frames and events, e.g. a DAVIS series camera.
```

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_tracking/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_tracking/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/feature_tracking/combined_lk_tracking.png
---
align: center
---
Tracked features on frame and event streams from a camera.
```

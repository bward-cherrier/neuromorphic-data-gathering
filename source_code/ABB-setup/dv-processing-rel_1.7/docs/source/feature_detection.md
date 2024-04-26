# Feature detection in events

The dv-processing library provides extensible interfaces and implementations for feature detection on event streams.
Features are certain locations of interest in the data that can be detected with repeatability, such as corners. The
detected features can be tracked over time in the event stream, so the library provides a reusable interface for feature
detection algorithm implementations that can be used with a tracking algorithm.

## Feature detectors

This section describes available feature detectors and their basic usage.

### Feature detector using OpenCV

The most basic use of a feature detector is to use OpenCV feature detector implementation with the dv-processing feature
detector wrapper on a {cpp:type}`dv::Frame`. The feature detection wrapper base class
{cpp:class}`dv::features::FeatureDetector` was designed to wrap a feature detection algorithm and provide a few
additional processing steps to handle image margins (ignore feature that are very close to edge of the image) and
subsampling as a post-detection step (sampling only an optimal set of features).

Margins are set by a floating-point coefficient value, which expresses a relative margin size from the resolution of an
input image, e.g. 0.05 would set the margins of 5% of the total width / height of the image.

Only a select number of features are useful for most applications. The {cpp:class}`dv::features::FeatureDetector`
simplifies feature subsampling by providing a post-processing step that sub-samples the output features. The
sub-sampling can be performed by enabling one the following post-processing options:

- `None`: Do not perform any post-processing, all features from detection will be returned.
- `TopN`: Retrieve a given number of the highest scoring features.
- `AdaptiveNMS`: Apply the AdaptiveNMS algorithm to retrieve equally spaced features in pixel space dimensions. More
  information on the AdaptiveNMS here: [original code](https://github.com/BAILOOL/ANMS-Codes) and
  [paper](https://www.researchgate.net/publication/323388062_Efficient_adaptive_non-maximal_suppression_algorithms_for_homogeneous_spatial_keypoint_distribution)
  .

The following code sample shows how to detect the "good features to track" from OpenCV on an accumulated image from a
live camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_detection/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_detection/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/feature_detection/detected_features.png
---
align: center
---
Detected "good features to track" on an accumulated image.
```

### Arc\* Feature detector

The library provides an implementation of Arc\* feature detector. Arc\* performs corner detection on a per-event basis,
so it uses {cpp:type}`dv::EventStore` as an input. More details on this feature detection algorithm can be found in the
[original paper publication](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/277131/RAL2018-camera-ready.pdf)
.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_detection/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_detection/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{note}
The provided implementation is not suitable for running real-time with high event rates.
```

```{figure} assets/feature_detection/arc_features.png
---
align: center
---
Detected Arc* features displayed on an accumulated image.
```

### Event-based blob detector

The library provides an implementation of a simple event-based blob detector. The class makes use of
`cv::SimpleBlobDetector` to detect blobs on an image. The image is generated using {cpp:type}`dv::EdgeMapAccumulator`. A
default `cv::SimpleBlobDetector` is provided with reasonable values to safely detect blobs and not noise in the
accumulated image. It is also possible to specify the detector that should be used to find the blobs and some specific
region of interests in the image plane where the blobs should be searched. In addition to this, eventually, it is also
possible to specify the down sampling factor to be applied to the image before performing the detection and also specify
any additional pre-processing step that should be applied before running the actual detection step. In summary, the
detection steps are as following:

1. Compute accumulated image from events
1. Apply ROI and mask to the accumulated event image
1. Down sample image (optional)
1. Apply pre-process function (optional)
1. Detect blobs
1. Rescale blobs to original resolution (if down sampling was performed)

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/feature_detection/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/feature_detection/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

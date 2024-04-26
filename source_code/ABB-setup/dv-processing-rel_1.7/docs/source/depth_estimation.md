# Depth estimation

Depth estimation with event cameras is possible by applying the same approach of disparity calculation on a calibrated
stereo camera rig. The straightforward approach is to accumulate frames from events on both cameras and use the same
disparity estimation algorithm. This approach might have some limitations, since accumulating events might result in
suboptimal results due to low texture available in an accumulated frame.

The dv-processing library provides the {cpp:class}`dv::camera::StereoGeometry` and a few disparity estimation algorithms
that, in combination, can be used to build a depth estimation pipeline.

## Semi-dense stereo block matching

Dense block matching here refers to the most straightforward approach: accumulating full frames and running a
conventional disparity estimation on top to estimate depth. Since the accumulated frames only contain limited texture
due to pixels reacting to brightness changes - this approach is referred to as semi-dense. The
{cpp:class}`SemiDenseStereoMatcher` class wraps the disparity estimation part, where estimated disparity can be used to
calculate depth with {cpp:class}`dv::camera::StereoGeometry`.

Following sample code show the use of {cpp:class}`SemiDenseStereoMatcher` with {cpp:class}`dv::camera::StereoGeometry`
to run a real-time depth estimation pipeline on a calibration stereo camera.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/depth_estimation/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/depth_estimation/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/depth_estimation/semi-dense.png
---
align: center
---
Expected result of semi-dense disparity estimation. The output provides two accumulated frames and
color-coded disparity map.
```

```{note}
Disparity map yields results only in areas with visible texture, areas without texture contain speckle noise.
```

## Sparse disparity estimation

The semi-dense appraoch is a straightforward to stereo disparity estimation. Another approach is to perform disparity
estimation on sparse selected regions within accumulated image. Sparse estimation approach allows the implementation to
select regions with enough texture to be selected for the disparity, reducing computational complexity and improving
quality. The sparse approach takes point coordinates of where the disparity needs to be estimated, performs sparse
accumulation only in the regions where disparity matching actually needs to happen and runs correlation based template
matching of left image patches on the right camera image. Each template is matched against the other image on a
horizontal line using normalized correlation coefficient (Pearson correlation) and the best scoring match is considered
to be the correct match and according disparity is assigned to that point.

The following sample code shows the use of sparse disparity block matcher with a live calibrated stereo camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/depth_estimation/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/depth_estimation/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/depth_estimation/sparse-disparity.png
---
align: center
---
Expected result of sparse disparity estimation. The colored rectangles represent sparse blocks that
are matched on the right side image. Block colors are matched on both images. Note that frame are sparse as
well - the accumulation happens only in relevant areas around points of interest. The points of interest
are selected on high density event areas as per mean-shift cluster extraction.
```

# Welcome to dv-processing's documentation!

Generic processing algorithms for event cameras.

# Introduction

This documentation describes the API and the algorithms available in the `dv-processing` library. The documentation
covers the basic usage of the library for event camera data processing. The library builds on top of C++20 coding
standard, provides state-of-the-art algorithms to process event data streams from iniVation cameras with high
efficiency. The library also provides Python bindings that allows users to develop high performance event processing
application in Python as well. Since the library is built on top of modern C++, it extensively uses template
metaprogramming to provide extensible and performant implementations of event processing algorithms. Python bindings
have some limitations due to the of use templates, but most applications can be developed using Python alone. Extensive
code samples in both C++ and Python are provided next to algorithm description, the samples also follow modern coding
style conventions. More information about the coding style convention can be found
[here](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md). Code samples apply the
[constant and immutability rules](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#S-const)
from the previously mentioned document.

# Getting started

This section covers installation and usage of the library in Linux {fa}`linux`, Windows {fa}`windows`, and MacOS
{fa}`apple`. Usage in CMake projects is covered for the C++ API, and pip installation is preferred for Python projects.

```{toctree}
---
caption: Getting started
maxdepth: -1
hidden:
---
installation.md
usage.md
```

# Basics

This section covers the very basic features of this library:

- Storing events in memory and efficiently accessing them;
- Frame image accumulation algorithms from events;
- Noise filtering in events and efficient subsampling;
- Input/Output of event data: live camera access, as well as reading / writing data from a file.

```{figure} assets/accumulator/edge_map.png
---
align: center
---
Frame generated using {cpp:class}`dv::EdgeMapAccumulator` class.
```

```{toctree}
---
maxdepth: -1
caption: Basics
hidden:
---
event_store.md
event_stream_slicing.md
accumulators.md
event_filtering.md
cli-utilities.md
reading_data.md
writing_data.md
network_streaming.md
```

# Vision algorithms

This chapter describes the available event processing algorithms that can are building blocks for computer vision with
event cameras. Most notable algorithms and features:

- Camera geometry - sensor calibration, pixel projections, lens undistortion operations;
- Feature detection and tracking
- Minimal kinematics routines
- Mean-shift clustering

```{figure} assets/feature_tracking/combined_lk_tracking.png
---
align: center
---
Tracked features on frame and event streams from a camera.
```

```{toctree}
---
maxdepth: -1
hidden:
caption: Vision algorithms
---
camera_geometry.md
feature_detection.md
feature_tracking.md
kinematics.md
mean_shift_clustering.md
mean_shift_tracker.md
```

# Advanced applications

This section describes the use of available library features and algorithms to build more complex applications and
algorithms for: depth estimation, motion compensation and contrast maximization.

```{toctree}
---
maxdepth: -1
hidden:
caption: Advanced applications
---
depth_estimation.md
motion_compensation.md
contrast_maximization.md
```

```{figure} assets/depth_estimation/semi-dense.png
---
align: center
---
Expected result of semi-dense disparity estimation. The output provides two accumulated frames and
color-coded disparity map.
```

# API

API documentation provides references for available classes and methods in the library. You can find detailed
documentation on each method for their use case, function arguments, and produced outputs.

```{toctree}
---
maxdepth: -1
hidden:
caption: API documentation
---
api.md
```

# Help

In case of technical issues or any problems, please visit the
[support page](https://docs.inivation.com/help/support.html). Any issues in the documentation or the code can be
reported to our [gitlab issue tracker](https://gitlab.com/inivation/dv/dv-processing/-/issues).

```{toctree}
---
maxdepth: -1
hidden:
caption: Help
---
Support <https://docs.inivation.com/help/support.html>
Report an issue <https://gitlab.com/inivation/dv/dv-processing/-/issues>
```

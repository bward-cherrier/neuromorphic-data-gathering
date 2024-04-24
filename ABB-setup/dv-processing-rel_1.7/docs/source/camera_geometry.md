# Camera Geometry

This section introduces camera calibration related classes and primitives available in the dv-processing library.

## Calibration files

Multiple calibration file formats were introduced in different DV software components. Single / stereo camera
calibration files were introduced in DV camera calibration module, which produces these files after a successful
calibration. A new yaml calibration file format is introduced in dv-processing, which can store calibration information
not only for a single / stereo camera setups, but multi-camera setups with any number of cameras as well as calibration
for IMU, and it's noise parameters.

All of these calibration files can be opened and created by a provided {cpp:class}`dv::camera::CalibrationSet`. The
`CalibrationSet` class contains three main types of calibration parameters:

- {cpp:class}`dv::camera::calibrations::CameraCalibration` - Single camera intrinsic calibration: pinhole camera and
  distortion model parameters. It also contains extrinsic transformation matrix which describes physical displacement of
  a camera w.r.t. one of the cameras. Usually first camera denoted "C0" is selected as a reference which will have an
  identity matrix for its transformation and all the other cameras will contain transformation matrices which describes
  the relationship to the first camera.
- {cpp:class}`dv::camera::calibrations::StereoCalibration` - Stereo camera extrinsic parameters, this class contains
  essential and fundamental matrices between a select pair of cameras.
- {cpp:class}`dv::camera::calibrations::IMUCalibration` - IMU extrinsic calibration and IMU noise parameters. This class
  contains extrinsic parameters - transformation between camera plane and timestamp offset (time calibration). It
  contains IMU measurement noise parameters: gyroscope and accelerometer measurement biases, noise densities, and random
  walk (white noise) parameters.

The {cpp:class}`dv::camera::CalibrationSet` contains sets of these three types of calibration parameters, which is
sufficient for storing parameters of any generic visual-inertial multi-camera rig calibration. To see the full list of
exact available parameters, please see the list of public members of each of the calibration classes.

### Writing a calibration file

The following code sample shows how to generate a calibration file given hardcoded calibration parameters. The generated
file is going to be used as input for the next section [](#reading-calibration-file).

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/camera_geometry/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/camera_geometry/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Reading calibration file

The following code sample shows the use of the {cpp:class}`dv::camera::CalibrationSet` to load an existing calibration
file.

```{note}
This sample requires an existing "calibration.json" file existing in the directory the binary is executed in. The file
can be obtained by running previous sample "Writing a calibration file".
```

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/camera_geometry/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/camera_geometry/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

## Monocular camera geometry

While {cpp:class}`dv::camera::CalibrationSet` class is designed for reading / writing the calibration parameters of
multi-camera rig and {cpp:class}`dv::camera::calibrations::CameraCalibration` stores intrinsic calibration parameters of
a single pinhole camera model, these classes do not provide any operations that perform projections and (un)distortion
of point coordinates. {cpp:class}`dv::camera::CameraGeometry` class provides an efficient implementation of the
mathematical operations for:

- Forward / backward projection,
- Distortion model distortion and undistortion.

```{note}
The operations are intended for sparse point operations, since event-camera produces parse pixel data. For operations
on full image frame prefer the use of highly optimized operations available in OpenCV.
```

The following code sample shows the use of {cpp:class}`dv::camera::CameraGeometry` to back-project pixel coordinates
onto 3D space, apply a 3D transformation on the points using the kinematics library and project the points back to pixel
coordinates.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/camera_geometry/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/camera_geometry/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/camera_geometry/reprojected_transform.png
---
align: center
---
Expected result of running the sample code for camera geometry transformation.
```

## Stereo camera geometry

The {cpp:class}`dv::camera::StereoGeometry` class is intended for efficient rectification of sparse point coordinates
and 3D depth estimation. The {cpp:class}`dv::camera::StereoGeometry` is built on top of monocular
{cpp:class}`dv::io::CameraGeometry` projections and adds sparse stereo rectification that aligns coordinates for stereo
disparity matching. The class also provides convenience methods to estimate 3D depth with known disparity.

For sample use of the stereo geometry class, please refer to full disparity estimation samples available
[here](https://gitlab.com/inivation/dv/dv-processing/-/tree/rel_1.6/samples/depth).

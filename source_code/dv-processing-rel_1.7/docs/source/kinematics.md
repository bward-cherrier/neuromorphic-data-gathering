# Kinematics primitives

The dv-processing library provides a minimal implementation of kinematic transformation primitives that are useful for
geometric computer vision algorithms. This includes transformations of 3D points using transformation matrices and
handling rigid-body motion trajectories over time. These basic primitives can be used to implement motion compensation
algorithm for event data, which can be used to reduce or eliminate motion blur in the event stream. The underlying
implementation uses mathematical operations from Eigen library, so the mathematical operations are expected to be highly
efficient.

## Transformation

A transformation in dv-processing describes an object's orientation and position in 3D space at a certain point in time.
Transformation contains a timestamp, rotational and translational transformation expressed as a 4x4 homogenous
transformation matrix T, like this:

```{math}
T = \begin{bmatrix}
r_0 & r_1 & r_2 & t_0 \\
r_3 & r_4 & r_5 & t_1 \\
r_6 & r_7 & r_8 & t_2 \\
0 & 0 & 0 & 1
\end{bmatrix} = \begin{bmatrix}
r & t \\
0 & 1
\end{bmatrix}
```

Here:

- {math}`r, r_{0..9}` - is a rotation matrix (and it's coefficients) that describes an object's rotation.
- {math}`t, t_{0..2}` - is a vector describing translational vector, which is an object's displacement.

A transformation matrix together with a timestamp describes complete attitude with 6 degrees of freedom. This
transformation can be applied to other transformations as well as 3D points to obtain a new relative position with the
applied transformation. The transformation is implemented in the {cpp:class}`dv::kinematics::Transformation` class,
which is a templated class. The template parameter sets the underlying matrix scalar data type for the 4x4 matrix, which
is either `float` or `double`. To simplify the use case, two predefined aliases are defined:
{cpp:type}`dv::kinematics::Transformationf` and {cpp:type}`dv::kinematics::Transformationd` - they differ in the
underlying scalar data type:

- `Transformationf` uses 32-bit single precision floating point values,
- `Transformationd` uses 64-bit double precision floating point values.

The library usually prefers the use of single precision floating point scalar type, since the representation is accurate
enough for sub-millimeter accuracy with lower memory footprint.

The following sample code shows how to initialize a transformation and apply it to a 3D point.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/kinematics/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/kinematics/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

## Linear transformer

A set of transformations that are monotonically increasing in time can be formed into a motion trajectory. Linear
transformer can be used to store a set of transformation representing a single objects trajectory and extract
transformations at specified points in time, which are calculated using linear interpolation between the nearest
available transformations.

The following sample code shows how to use the {cpp:type}`dv::kinematics::LinearTransformerf` class to interpolate
intermediate transformations in time:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/kinematics/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/kinematics/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

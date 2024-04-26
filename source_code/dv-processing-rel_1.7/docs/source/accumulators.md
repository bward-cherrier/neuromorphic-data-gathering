# Event accumulation

Events are a sparse representation of brightness changes measured by the pixels on a camera sensor. Accumulation is a
method applied to events to generate a frame representation of events. The dv-processing library provides a few highly
optimized algorithm implementations to perform event accumulation and achieve different frame-like representation of the
events.

## Definitions

Certain definitions are used in this chapter and within accumulator API. Following is a list of specific definitions and
their meaning within context of the accumulator:

- Potential - normalized pixel brightness value in some floating point range; it is an internal representation range for
  brightness that can be scaled into other brightness representations;
- Contribution - a numeric value that an event contributes to the brightness of pixel;
- Decay - pixel brightness correction applied over time when no events contribute to its brightness;
- Neutral potential - a default pixel brightness without any contribution;
- Minimum / maximum potential - limits for potential value representation.

## Accumulator

The {cpp:type}`dv::Accumulator` is a generalized implementation of a few accumulation algorithms that can be configured
using class methods. Following chapter will describe the configuration options available for the accumulator. Final
chapter of this section will provide a code sample that shows the use of all these configuration options for
accumulation of events from a live camera.

### Decay function and decay param

One of *None*, *Linear*, *Exponential*, *Step*. Defines the data degradation function that should be applied to the
image. For each function, the *Decay param* setting assumes a different function:

| Function    | Enum value           | *Decay param* function                                                | Explanation                                                                                                                                                                                                                       |
| ----------- | -------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| None        | `Decay::NONE`        | No function                                                           | Does not apply any decay                                                                                                                                                                                                          |
| Linear      | `Decay::LINEAR`      | The slope `a` of the linear function, in *intensity per microsecond*  | Assume intensity of a pixel is `I0` at time `0`, this function applies a linear decay of the form of <br/>`I0 - (t * decayparam)` or  <br/>`I0 + (t * decayparam)`until the value hits the value specified in *neutral potential* |
| Exponential | `Decay::EXPONENTIAL` | The time constant `tau` of the exponential function in *microseconds* | Assume intensity of a pixel is `I0` at time `0`, this function applies an exponential decay of the form <br/>`I0 * exp(-(t/decayparam))`. The decay approaches a value of `neutralPotential` over time.                           |
| Step        | `Decay::STEP`        | No function                                                           | Set all pixel values to *neutral potential* after a frame is extracted.                                                                                                                                                           |

The decay function can be set using {cpp:func}`dv::Accumulator::setDecayFunction()` method.

### Event Contribution

The contribution an event has onto the image. If an event arrives at a position `x`, `y`, the pixel value in the frame
at `x`, `y` gets increased / decreased by the value of *Event contribution*, based on the events polarity.

Except:

- The resulting pixel value would be higher than *Max potential*, the value gets set to *Max potential* instead
- The resulting pixel value would be lower than *Min potential*, the value gets set to *Min potential* instead
- The event polarity is negative, and *Ignore polarity* is enabled, then the event is counted positively

Event contribution can be set using the {cpp:func}`dv::Accumulator::setEventContribution()` method.

### Min potential / Max potential

Sets the minimum and maximum values a pixel can achieve. If the value of the pixel would reach higher or lower, it is
capped at these values. These values are also used for normalization at the output. The frame the module generates is an
`unsigned 8-bit` grayscale image, normalized between *Min potential* and *Max potential*. A pixel with the value *Min
potential* corresponds to a pixel with the value `0` in the output frame. A pixel with the value *Max potential*
corresponds to a pixel with the value `255` in the output frame.

*Min potential* and *Max potential* can be set using {cpp:func}`dv::Accumulator::setMinPotential()` and
{cpp:func}`dv::Accumulator::setMaxPotential()` methods.

### Neutral potential

This setting has different effects depending on the decay function:

| Function    | *Neutral potential* function                                               |
| ----------- | -------------------------------------------------------------------------- |
| None        | No function.                                                               |
| Linear      | Pixel brightness value decays linearly into the *neutral potential* value. |
| Exponential | No function.                                                               |
| Step        | Each pixel value is set to *neutral potential* after generating a frame.   |

*Neutral potential* can be set using {cpp:func}`dv::Accumulator::setNeutralPotential()`.

### Ignore polarity

If this value is set, all events act as if they had positive polarity. In this case, *Event contribution* is always
taken positively. This can be used to generate edge images instead of an actual image reconstruction. This can be done
by setting *Neutral potential* and *Min potential* to zeros.

This feature can be enabled using the {cpp:func}`dv::Accumulator::setIgnorePolarity()` method.

### Synchronous Decay

If this value is set, decay happens in continuous time for all pixels. In every frame, each pixel will be eagerly
decayed to the time the image gets generated. If this value is not set, decay at the individual pixel only happens when
the pixel receives an event. Decay is lazily evaluated at the pixel.

```{note}
Both decay regimes yield the same overall decay over time, just the time at which it is applied changes. This
parameter does not have an effect for Step decay. Step decay is always synchronous at generation time.
```

Synchronous decay can be enabled using the {cpp:func}`dv::Accumulator::setSynchronousDecay()` method.

### Accumulating event from a camera

The following sample code show how to use {cpp:class}`dv::Accumulator` together with {cpp:type}`dv::EventStreamSlicer`
and {cpp:class}`dv::io::CameraCapture` to implement a pipeline that generates continuous stream of accumulated frames:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/accumulators/sample6.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/accumulators/sample6.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/accumulator/accumulator.png
---
align: center
---
Frame generated using {cpp:class}`dv::Accumulator` class.
```

## Edge accumulation

The dv-processing library provides a highly-optimized variant of accumulator for generating edge maps -
{cpp:class}`dv::EdgeMapAccumulator`. It was specifically optimizes for speed of execution, so it has only a minimal set
of settings and supported features compared to {cpp:class}`dv::Accumulator`.

Below is a table providing available parameters for the {cpp:class}`dv::EdgeMapAccumulator`:

| Parameter         | Default value | Accepted values | Comment                                                                                                                                                                                                                                                                                                                                                                                      |
| ----------------- | ------------- | --------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Contribution      | 0.25          | \[0.0; 1.0\]    | Contribution potential for a single event.                                                                                                                                                                                                                                                                                                                                                   |
| Ignore polarity   | true          | boolean         | All events are considered positive if enabled.                                                                                                                                                                                                                                                                                                                                               |
| Neutral potential | 0.0           | \[0.0; 1.0\]    | Neutral potential is the default pixel value when decay is disabled and the value that pixels decay into when decay is enabled.                                                                                                                                                                                                                                                              |
| Decay param       | 1.0           | \[0.0; 1.0\]    | This value defines how fast pixel values decay to neutral value. The bigger the value the faster the pixel value will reach neutral value. Decay is applied before each frame generation. The range for decay value is \[0.0; 1.0\], where 0.0 will not apply any decay and 1.0 will apply maximum decay value resetting a pixel to neutral potential at each generation (default behavior). |

Following sample show the use of {cpp:class}`dv::EdgeMapAccumulator` with {cpp:type}`dv::EventStreamSlicer` and
{cpp:class}`dv::io::CameraCapture` to generate stream of edge images:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/accumulators/sample7.cpp
:language: c++
:linenos:
:tab-width: 4
```
````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/accumulators/sample7.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/accumulator/edge_map.png
---
align: center
---
Frame generated using {cpp:class}`dv::EdgeMapAccumulator` class.
```

## Event visualization

Accumulators, described in previous chapters are useful when image or edge representation of events is needed, and they
are mostly useful to process events using typical image processing algorithms.
{cpp:class}`dv::visualization::EventVisualizer` class serves a purpose to perform simple event visualization. Instead of
increasing or decreasing pixel brightness, the {cpp:class}`dv::visualization::EventVisualizer` just performs color
coding of pixel coordinates where an event was registered.

Following sample show the use of {cpp:class}`dv::visualization::EventVisualizer` class to generate colored previews of
events using {cpp:type}`dv::EventStreamSlicer` and {cpp:class}`dv::io::CameraCapture`:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/accumulators/sample8.cpp
:language: c++
:linenos:
:tab-width: 4
```
````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/accumulators/sample8.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/accumulator/visualizer.png
---
align: center
---
Frame generated using {cpp:class}`dv::visualization::EventVisualizer` class.
```

## Time surface

Time surface is an event representation in an image frame, except instead of representing event in pixel brightness, it
represents event in a 2D image structure, but pixel value contains the latest event timestamp.

The timestamps representations can be normalized to retrieve an image representation of the time surface. It will
represent the latest timestamps with the brightest pixel values.

Following sample show the use of {cpp:type}`dv::TimeSurface` class to generate time surface previews of events using
{cpp:type}`dv::EventStreamSlicer` and {cpp:class}`dv::io::CameraCapture`:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/accumulators/sample9.cpp
:language: c++
:linenos:
:tab-width: 4
```
````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/accumulators/sample9.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/accumulator/time_surface.png
---
align: center
---
Frame generated using {cpp:type}`dv::TimeSurface` class.
```

## Speed invariant time surface

Speed invariant time surface is a specific time surface variant that is more suitable for feature extraction, the
implementation follows this paper: https://arxiv.org/pdf/1903.11332.pdf.

Following sample show the use of {cpp:type}`dv::SpeedInvariantTimeSurface` class to generate time surface previews of
events using {cpp:type}`dv::EventStreamSlicer` and {cpp:class}`dv::io::CameraCapture`:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/accumulators/sample10.cpp
:language: c++
:linenos:
:tab-width: 4
```
````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/accumulators/sample10.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/accumulator/speed_invariant_time_surface.png
---
align: center
---
Frame generated using {cpp:type}`dv::SpeedInvariantTimeSurface` class.
```

## Performance of available accumulators

The library performs benchmarking of available accumulation algorithms to ensure their best performance. Accumulators
are also benchmarked on two metrics:

- Event throughput - measured in millions of (mega) events per second;
- Framerate - measured in generated frames per second.

The benchmarks are performed by generating a batch of events at uniformly random pixel coordinates on a VGA (640x480)
resolution. Below are the results of running the benchmark on AMD Ryzen 7 3800X 8-Core Processor:

| Accumulator type          | Framerate (FPS) | Throughput (MegaEvent/s) |
| ------------------------- | --------------- | ------------------------ |
| Accumulator               | 668             | 66.5                     |
| EdgeMapAccumulator        | 1767            | 149.1                    |
| EventVisualizer           | 785             | 78.3                     |
| TimeSurface               | 910             | 91.5                     |
| SpeedInvariantTimeSurface | 370             | 36.8                     |

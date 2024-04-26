# Reading camera data

The dv-processing library provides a convenient way of reading camera data from live connected cameras and persistent
files.

## Camera name

Camera name is used to identify a unique camera produced by iniVation. Camera name consists of a camera model and a
serial number, concatenated by an underscore ("\_") character. The library refers to camera name in multiple methods,
this value can be consistently used across the library.

Some examples of a camera name:

- DVXplorer: `DVXplorer_DXA00093`, `DVXplorer_DXM00123`
- DAVIS: `DAVIS346_00000499`

```{note}
This definition is valid for USB cameras, the camera name is also reported in network streaming sources. In that case,
camera name can be manually set by the developer, so naming convention for the models might not be entirely followed.
```

## From a camera

The easiest approach to access data from a live camera is to use the {cpp:class}`dv::io::CameraCapture` class. This
section provides in-depth explanation on the usage and code samples.

### Discover connected cameras

The camera name can be inspected using a command-line utility [dv-list-devices](cli-utilities.md#dv-list-devices) that
is available in the packages of dv-processing. Sample output for the utility:

```
$ dv-list-devices
Device discovery: found 2 devices.
Detected device [DAVIS346_00000499]
Detected device [DVXplorer_DXA00093]
```

Device discovery is also possible with the use of library methods. Following is a sample on how to detect connected
devices using discovery method:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Opening a camera

The {cpp:class}`dv::io::CameraCapture` class follows
[RAII](https://en.wikipedia.org/wiki/Resource_acquisition_is_initialization) pattern for resource management. Creating
an instance of the class will open the camera connected on USB and starts reading the data immediately, the resources
are released when the object instance is destroyed.

The constructor of this class accepts two arguments: camera name \[string\] and camera type \[enum\] that are used to
specify which camera needs to be opened. The default argument values are designed to not constrain the camera
specification and effectively opens first detected camera in the system.

`````{tabs}
````{group-tab} C++
```c++
#include <dv-processing/io/camera_capture.hpp>

// Open first detected camera in the system
dv::io::CameraCapture capture;
```
````

````{group-tab} Python
```python
import dv_processing as dv

capture = dv.io.CameraCapture()
```
````
`````

It's also possible to open a specific camera on the system, by providing a camera name:

`````{tabs}
````{group-tab} C++
```c++
// Open the specified camera
dv::io::CameraCapture capture("DVXplorer_DXA000000");
```
````

````{group-tab} Python
```python
import dv_processing as dv

# Open the specified camera
capture = dv.io.CameraCapture(cameraName="DVXplorer_DXA000000")
```
````
`````

Camera type argument can be used to open a camera of given type. If both parameters are provided, the camera will need
to match both field requirements to be opened by the {cpp:class}`dv::io::CameraCapture` class:

`````{tabs}
````{group-tab} C++
```c++
// Open any DAVIS camera (camera name not specified)
dv::io::CameraCapture capture("", dv::io::CameraCapture::CameraType::DAVIS);
```
````

````{group-tab} Python
```python
import dv_processing as dv

# Open any DAVIS camera (camera name not specified)
capture = dv.io.CameraCapture(type=dv.io.CameraCapture.CameraType.DAVIS)
```
````
`````

### Checking camera capabilities

The {cpp:class}`dv::io::CameraCapture` class abstracts all cameras manufactured by iniVation, since some camera provide
different data types, the capture class provides methods to test what data the camera can provide:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Configuring camera options

Some advanced properties of our cameras can be configured by a number of functions. They are listed here for reference,
please check their detailed API documentation for more details.

#### DVXplorer camera advanced control functions:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample16.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample16.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

Read more about DVXplorer biases in our
[documentation page](https://docs.inivation.com/hardware/hardware-advanced-usage/biasing.html#dvxplorer-biases) and
specific details about `eFPS` implementation in {fa}`gitlab`
[dv-processing source code](https://gitlab.com/inivation/dv/dv-processing/-/blob/rel_1.7/include/dv-processing/io/camera_capture.hpp?ref_type=heads#L946-1038).

```{note}
On DVXplorer, setting global hold to false can help for certain applications containing
repeating patterns observation, such as flickering LEDs.
```

#### DAVIS camera advanced control functions:

- General options:

  `````{tabs}
  ````{group-tab} C++
  ```{literalinclude} assets/code_samples/c++/reading_data/sample17.cpp
  :language: c++
  :linenos:
  :tab-width: 4
  ```
  ````

  ````{group-tab} Python
  ```{literalinclude} assets/code_samples/python/reading_data/sample17.py
  :language: python
  :linenos:
  :tab-width: 4
  ```
  ````
  `````

- Frame options:

  `````{tabs}
  ````{group-tab} C++
  ```{literalinclude} assets/code_samples/c++/reading_data/sample18.cpp
  :language: c++
  :linenos:
  :tab-width: 4
  ```
  ````

  ````{group-tab} Python
  ```{literalinclude} assets/code_samples/python/reading_data/sample18.py
  :language: python
  :linenos:
  :tab-width: 4
  ```
  ````
  `````

- Event options (biases):

  ```{warning}
  Before using biases, make sure that you absolutely need to change them and that you understand them by reading about
  biases on our [documentation page](https://docs.inivation.com/hardware/hardware-advanced-usage/biasing.html).
  ```

  `````{tabs}
  ````{group-tab} C++
  ```{literalinclude} assets/code_samples/c++/reading_data/sample19.cpp
  :language: c++
  :linenos:
  :tab-width: 4
  ```
  ````

  ````{group-tab} Python
  ```{note}
  Hard-coded bias addresses come from their definition in libcaer as seen {fa}`gitlab`
  [here](https://gitlab.com/inivation/dv/libcaer/-/blob/master/include/libcaer/devices/davis.h#L1334-1372).
  ```
  ```{note}
  Conversion utilities `caer_bias_coarse_fine_generate`, `caer_bias_coarse_fine_parse` and `CaerBiasCoarseFine`
  implementations don't exist in Python since they originally come from libcaer.
  ({fa}`gitlab` [struct source code](https://gitlab.com/inivation/dv/libcaer/-/blob/master/include/libcaer/devices/davis.h#L1649-1662),
  {fa}`gitlab` [functions source code](https://gitlab.com/inivation/dv/libcaer/-/blob/master/src/davis.c#L805-840))

  They are therefore provided as part of this sample.
  ```
  ```{literalinclude} assets/code_samples/python/reading_data/sample19.py
  :language: python
  :linenos:
  :tab-width: 4
  ```
  ````
  `````

### Read events from a live camera

Incoming data from a camera can be read sequentially using the {cpp:func}`dv::io::CameraCapture::getNextEventBatch()`.
Following is a minimal sample on how to read events sequentially from a camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample3.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample3.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read frames from a live camera

Incoming frames from a camera can be read sequentially frame-by-frame using the
{cpp:func}`dv::io::CameraCapture::getNextFrame()`. Following is a minimal sample on how to read frames sequentially from
a camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample4.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample4.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read IMU data from a live camera

Incoming imu data from a camera can be read sequentially using the {cpp:func}`dv::io::CameraCapture::getNextImuBatch()`.
Following is a minimal sample on how to read imu data sequentially from a camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample5.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample5.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read triggers from a live camera

```{note}
To understand what triggers are and where they come from, read more about them on our
[documentation page](https://docs.inivation.com/hardware/hardware-advanced-usage/external-camera-sync.html).
```

Incoming trigger data from a camera can be read sequentially using the
{cpp:func}`dv::io::CameraCapture::getNextTriggerBatch()`. Following is a minimal sample on how to read trigger data
sequentially from a camera:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample6.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample6.py
:language: python
:linenos:
:tab-width: 4
```

```{note}
Hard-coded values come from their corresponding definition (as seen in C++ tab) in libcaer source code:
- for DVXplorer, search for 'DVX_EXTINPUT' and '_DETECT' in
{fa}`gitlab` [here](https://gitlab.com/inivation/dv/libcaer/-/blob/master/include/libcaer/devices/dvxplorer.h).
- for Davis, search for 'DAVIS_CONFIG_EXTINPUT' and '_DETECT' in
{fa}`gitlab` [here](https://gitlab.com/inivation/dv/libcaer/-/blob/master/include/libcaer/devices/davis.h).
```
````
`````

### Sample application - reading data from a live camera

An application reading multiple types of data from a live camera can be found among the code samples in the source code
repository of the dv-processing library:

- [Camera capture in C++](https://gitlab.com/inivation/dv/dv-processing/-/blob/rel_1.6/samples/io/camera-capture)
- [Camera capture in Python](https://gitlab.com/inivation/dv/dv-processing/-/blob/rel_1.6/python/samples/camera_capture.py)

## From a file

Data from iniVation cameras are usually recorded using the AEDAT4 file format. The dv-processing library provide tools
for reading such files. This section contains explanations and samples on how data can be read from AEDAT4 files. More
detailed information on the AEDAT4 file format can be found
[here](https://docs.inivation.com/software/software-advanced-usage/file-formats/aedat-4.0.html).

### Inspecting AEDAT4 files

AEDAT4 file format supports recording of different data streams into single file, multiple cameras are also supported.
The library provides a command-line utility for inspection of AEDAT4 files [dv-filestat](cli-utilities.md#dv-filestat),
it provides information on available streams recorded in it.

The utility provides information on the size, timestamp information, duration. More information about the utility can be
found [here](cli-utilities.md#dv-filestat).

### Opening a file

AEDAT4 files can be opened and read using a {cpp:class}`dv::io::MonoCameraRecording` class. This class assumes that the
recording was performed using a single camera. Following is a minimal sample code on opening a recording and printing
information about it.

A file can be opened by providing its path in the filesystem:

`````{tabs}
````{group-tab} C++

```{literalinclude} assets/code_samples/c++/reading_data/sample8.cpp
:language: c++
:linenos:
:tab-width: 4
```


````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample8.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Checking available streams

Data recordings might contain various data streams. The {cpp:class}`dv::io::MonoCameraRecording` provides easy-to-use
methods to inspect what data streams are available. Following sample code shows how to check for existence of various
data streams:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample9.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample9.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The dv-processing library also supports recording of other types, type agnostic methods are available as templated
methods in C++, while python only contains a limited set of named methods (since templating is unavailable in python).
Following sample show the use of generic method for checking the availability of certain streams with a name and a type:

```{literalinclude} assets/code_samples/c++/reading_data/sample14.cpp
---
language: c++
linenos:
tab-width: 4
---
```

### Read events from a file

Following sample reads events in batches while the stream has available data to read. While reading from a file, the
{cpp:func}`dv::io::MonoCameraRecording::getNextEventBatch()` will return data until the end of stream is reached, the
{cpp:func}`dv::io::MonoCameraRecording::isRunning()` method will return a false boolean when the end is reached.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample10.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample10.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read frames from a file

Following sample reads frames in batches while the stream has available data to read. While reading from a file, the
{cpp:func}`dv::io::MonoCameraRecording::getNextFrame()` will return a frame until the end of stream is reached, the
{cpp:func}`dv::io::MonoCameraRecording::isRunning()` method will return a false boolean when the end is reached.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample11.cpp
:language: c++
:linenos:
:tab-width: 4
```

````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample11.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read IMU data from a file

Following sample reads imu data in batches while the stream has available data to read. While reading from a file, the
{cpp:func}`dv::io::MonoCameraRecording::getNextImuBatch()` will return an IMU measurement batch until the end of stream
is reached, the {cpp:func}`dv::io::MonoCameraRecording::isRunning()` method will return a false boolean when the end is
reached.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample12.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample12.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Read triggers from a file

Following sample reads triggers in batches while the stream has available data to read. While reading from a file, the
{cpp:func}`dv::io::MonoCameraRecording::getNextTriggerBatch()` will return an IMU measurement batch until the end of
stream is reached, the {cpp:func}`dv::io::MonoCameraRecording::isRunning()` method will return a false boolean when the
end is reached.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/reading_data/sample13.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/reading_data/sample13.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### \[Advanced\] Reading custom data types

The previous samples show how to use named functions to read different data types. C++ API provides templated methods to
read any type of data. Below is sample that shows how to read data using the generic templated API:

```{note}
Since templated methods are only available in C++, the generic writing methods are only available in the C++ API.
```

```{literalinclude} assets/code_samples/c++/reading_data/sample15.cpp
---
language: c++
linenos:
tab-width: 4
---
```

### Sample application - reading data from a recorded AEDAT4 file

An application reading multiple types of data from an AEDAT4 file can be found in the source code repository of the
dv-processing library:

- [AEDAT4 player in C++](https://gitlab.com/inivation/dv/dv-processing/-/blob/rel_1.6/samples/io/aedat4-reader)
- [AEDAT4 to CSV converter in Python](https://gitlab.com/inivation/dv/dv-processing/-/blob/rel_1.6/python/samples/aedat4_reader.py)

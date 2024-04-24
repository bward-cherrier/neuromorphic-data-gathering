# Command-line utilities

The dv-processing library provides certain command-line utilities that are useful for troubleshooting basic issues or
performing some basic tasks.

## dv-filestat

The `dv-filestat` utility is useful for inspection of AEDAT4 files. It provides information on the content of a given
file. Usage:

```shell
$ dv-filestat path/to/file.aedat4
```

Sample output from this utility:

```shell
$ dv-filestat ~/dvSave-2022_06_29_13_40_44.aedat4
File path (canonical): "/home/rokas/dvSave-2022_06_29_13_40_44.aedat4"
File size (OS): 25429285
File size (Parser): 25429285
Compression: LZ4
Timestamp lowest: 1656502844057775
Timestamp highest: 1656502852007717
Timestamp difference: 7949942
Timestamp shift: 1656502844057775
Stream 0: events - EVTS
Stream 2: imu - IMUS
Stream 3: triggers - TRIG
DataTable file position: 25377171
DataTable file size: 52114
DataTable elements: 1590
```

Here, the provided fields are:

- File path - absolute file path in the filesystem;
- File size - actual file size in bytes;
- Compression - compression type;
- Timestamp - timing information in Unix microsecond format;
- Stream - available streams (e.g. "Stream 0: events - EVTS"), where "0" - stream id, "events" - stream name, "EVTS" -
  stream type identifier;
- DataTable - internal data layout information.

It is possible to pass a verbose flag ("-v" or "--verbose") to obtain more information on the recorded data. Sample
verbose output:

```shell
$ dv-filestat -v ~/dvSave-2022_06_29_13_40_44.aedat4
File path (canonical): "/home/rokas/dvSave-2022_06_29_13_40_44.aedat4"
File size (OS): 25429285
File size (Parser): 25429285
Compression: LZ4
Timestamp lowest: 1656502844057775
Timestamp highest: 1656502852007717
Timestamp difference: 7949942
Timestamp shift: 1656502844057775
Stream 0: events - EVTS
XML content:
0/
compression = LZ4
originalModuleName = capture
originalOutputName = events
typeDescription = Array of events (polarity ON/OFF).
typeIdentifier = EVTS
  0/info/
  sizeX = 640
  sizeY = 480
  source = DVXplorer_DXA00093
  tsOffset = 1656502833725006

Stream 2: imu - IMUS
XML content:
2/
compression = LZ4
originalModuleName = capture
originalOutputName = imu
typeDescription = Inertial Measurement Unit data samples.
typeIdentifier = IMUS
  2/info/
  source = DVXplorer_DXA00093
  tsOffset = 1656502833725006

Stream 3: triggers - TRIG
XML content:
3/
compression = LZ4
originalModuleName = capture
originalOutputName = triggers
typeDescription = External triggers and special signals.
typeIdentifier = TRIG
  3/info/
  source = DVXplorer_DXA00093
  tsOffset = 1656502833725006

DataTable file position: 25377171
DataTable file size: 52114
DataTable elements: 1590
Packet at 2342: StreamID 0 - Size 31375 - NumElements 4911 - TimestampStart 1656502844057775 - TimestampEnd 1656502844067773
Packet at 33725: StreamID 2 - Size 401 - NumElements 8 - TimestampStart 1656502844057878 - TimestampEnd 1656502844066774
Packet at 34134: StreamID 0 - Size 30591 - NumElements 4815 - TimestampStart 1656502844067783 - TimestampEnd 1656502844077744
Packet at 64733: StreamID 2 - Size 374 - NumElements 8 - TimestampStart 1656502844068045 - TimestampEnd 1656502844076941
Packet at 65115: StreamID 0 - Size 32736 - NumElements 5131 - TimestampStart 1656502844077777 - TimestampEnd 1656502844087761
Packet at 97859: StreamID 2 - Size 378 - NumElements 8 - TimestampStart 1656502844078212 - TimestampEnd 1656502844087107
Packet at 98245: StreamID 0 - Size 31442 - NumElements 4933 - TimestampStart 1656502844087779 - TimestampEnd 1656502844097756
Packet at 129695: StreamID 2 - Size 378 - NumElements 8 - TimestampStart 1656502844088378 - TimestampEnd 1656502844097274
```

The verbose output now prints each streams metadata, e.g.:

```
Stream 0: events - EVTS
XML content:
0/
compression = LZ4
originalModuleName = capture
originalOutputName = events
typeDescription = Array of events (polarity ON/OFF).
typeIdentifier = EVTS
  0/info/
  sizeX = 640
  sizeY = 480
  source = DVXplorer_DXA00093
  tsOffset = 1656502833725006
```

The XML metadata provide information on the compression type, some information on the type, as well as source camera
name and the resolution of the data.

Additionally, packet information is printed, e.g.:

```
Packet at 2342: StreamID 0 - Size 31375 - NumElements 4911 - TimestampStart 1656502844057775 - TimestampEnd 1656502844067773
```

The information here represents:

- "Packet at 2342" - the 2342 is the byte start index of this packet in the file;
- "StreamID 0" - id of the stream, which matches stream ids from output above;
- "Size 31375" - size of the content of this packet in bytes;
- "NumElements 4911" - number of element in packet, since it's an event packet, it contains 4911 events;
- "TimestampStart 1656502844057775 - TimestampEnd 1656502844067773" - this is the timing information of this packet, it
  contains data from time 1656502844057775 to 1656502844067773. These are Unix microsecond timestamps.

## dv-list-devices

The `dv-list-devices` is a basic utility that allows inspection of connected iniVation cameras on the system. Sample
output from this command:

```shell
$ dv-list-devices
Device discovery: found 1 devices.
Detected device [DVXplorer_DXA00093]
```

Running the command without any additional flags will just print the list of available cameras by their names. By adding
a verbose flag ("-v" or "--verbose"), it will print additional information about the available cameras, a sample output:

```shell
$ dv-list-devices --verbose
Device discovery: found 1 devices.
Detected device [DVXplorer_DXA00093]
- DVXplorer (type 8)
	- USB busNum:devAddr: 6:14
	- Device can be opened: true
	- USB serial number: DXA00093
	- Device needs firmware update: false
	- Timestamp Master: true
	- Firmware Version: 8
	- Logic Version: 18
	- Chip ID: 20
	- DVS Size X: 640
	- DVS Size Y: 480
	- DVS Statistics: true
	- External IO Generator: false
	- Multiplexer Statistics: true
	- IMU Model: Bosch BMI160
```

## dv-imu-bias-estimation

The `dv-imu-bias-estimation` tool is used to estimate intrinsic measurement biases that are measured by the IMU device
when there is no motion to be measured. The biases are internal offsets in the measuring device that come from
imperfections during manufacturing process of the IMU device.

Usage:

```shell
Usage: dv-imu-bias-estimation [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -c,--camera-name TEXT       Provide camera name to open. Application will open first discovered camera if a name is not provided.
  -t,--variance-threshold FLOAT [0.1]
                              Maximum variance that can be measured on IMU data. This value is used to determine motion in the data.
  -d,--duration FLOAT [1]     Time duration for collecting sample data in seconds.
  -a,--calibration-file TEXT:FILE
                              Path to the calibration file to store the biases values.
```

To estimate the biases, first place the camera steady on a level surface and run the utility:

```shell
$ dv-imu-bias-estimation
Opened camera [DVXplorer_DXA00093]
Keep the camera steady on a level surface and press ENTER to start data collection ...

Collected 794 samples over 1.007546 seconds
Bias estimation successful!
Accelerometer biases [x, y, z] in m/s^2:
	[-0.4947727, -0.7454767, 0.12935007]
Gyroscope biases [x, y, z] in rad/s:
	[0.0028670905, 0.004207727, 0.0023824223]
```

```{figure} assets/cli-utilities/level_surface.jpg
---
align: center
---
Camera placed on a level table surface for imu bias estimation.
```

When running without any parameters, the utility will open the first discovered camera and will perform bias estiamtion
for it. It will prompt the user to place the camera on a level surface and press enter key to start collection. When the
key is pressed, it will collect 1 second of IMU measurements and estimate the biases which are printed into the screen.
These values can be subtracted from raw measurements of IMU to compensate for the intrinsic offsets the IMU is
measuring.

The utility also provides additional parameters, below is a detailed explanation of these value:

- `-c,--camera-name` - provide a specific camera name to open. By default, the application will open first discovered
  device on the system, by providing a specific name, it will open the specified device.
- `-t,--variance-threshold` - to ensure that the camera is stationary while collecting data, the utility is measuring
  variance on the measured data. It will throw an exception if variance of at least one measurement is exceeding the
  given threshold. If this threshold is exceeded and the camera was not moved, this value can be increased to bypass
  this check.
- `-d,--duration` - duration of data collection in seconds. Usually one second is sufficient for a general case. The
  duration value can be tuned if estimated biases are inconsistent.
- `-a,--calibration-file` - the estimated bias values are only printed to the terminal output. It is also possible to
  save them in a calibration file for persistence. The utility will open the given file (the file needs to exist and
  contain a calibration) and update the IMU parameter values. If an IMU calibration for the camera already exists, it
  will overwrite previous settings, if IMU calibration for this camera does not exist, the calibration will be added.

## dv-tcpstat

The `dv-tcpstat` utility is useful for inspection of remote TCP server streams. It provides information on the stream
the remote server provides. It is compatible with TCP remote streaming instances created using
{cpp:class}`dv::io::NetworkWriter` class and with DV's
[Output net tcp server module](https://docs.inivation.com/software/dv/modules/built-in-modules/output-network.html#output-tcp-module).

Usage:

```shell
$ dv-tcpstat
Connect to a TCP streaming server and print information about it
Usage: dv-tcpstat [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -v,--verbose                Print full packet table
  -r,--try-reading            Try reading and printing packet information
  -i,--ip TEXT:IPV4 REQUIRED  IP address of the server to connect to
  -p,--port UINT:INT in [0 - 65535] REQUIRED
                              Server port number
```

As a sample demonstration of usage, let's take a DV project that outputs event stream into the output TCP module.

```{figure} assets/cli-utilities/dv_tcp_output_project.png
---
align: center
---
A project in DV running a live camera connected to a TCP output server module.
```

The module is configured to listen on IP address `127.0.0.1` and port `46581`, we can inspect this server using the
`dv-tcpstat` utility from the same computer. When running the utility with given IP, port, and verbose output flag, it
prints the information about the available data:

```shell
$ dv-tcpstat -i 127.0.0.1 -p 46581 -v
Attempting to connect to [127.0.0.1:46581]...
Connected to [127.0.0.1:46581]!

Stream info on stream ID 0:
        Stream name: "tcpserver"
        Stream type identifier: "EVTS"
        Stream details:
                /0/
                originalOutputName = tcpserver
                typeDescription = Array of events (polarity ON/OFF).
                typeIdentifier = EVTS
                                /0/info/
                                sizeX = 640
                                sizeY = 480
                                source = DVXplorer_DXA00093
```

The executable will print this information and disconnect immediately. It is possible to try and read some data from the
remote server by passing the `-r,--try-reading` flag, this is the expected with this option enabled:

```shell
$ dv-tcpstat -i 127.0.0.1 -p 46581 -r
Attempting to connect to [127.0.0.1:46581]...
Connected to [127.0.0.1:46581]!

Stream info on stream ID 0:
        Stream name: "tcpserver"
        Stream type identifier: "EVTS"
        Stream details:
                /0/
                originalOutputName = tcpserver
                typeDescription = Array of events (polarity ON/OFF).
                typeIdentifier = EVTS
                                /0/info/
                                sizeX = 640
                                sizeY = 480
                                source = DVXplorer_DXA00093
[2022-11-25 09:31:33] Received: EventStore containing 3103 events within 29416µs duration; time range within [1669365093787516; 1669365093816932]
[2022-11-25 09:31:33] Received: EventStore containing 3100 events within 29007µs duration; time range within [1669365093816932; 1669365093845939]
[2022-11-25 09:31:33] Received: EventStore containing 3105 events within 29034µs duration; time range within [1669365093845939; 1669365093874973]
[2022-11-25 09:31:33] Received: EventStore containing 3107 events within 28213µs duration; time range within [1669365093874973; 1669365093903186]
[2022-11-25 09:31:33] Received: EventStore containing 3090 events within 29441µs duration; time range within [1669365093903186; 1669365093932627]
[2022-11-25 09:31:33] Received: EventStore containing 3097 events within 29614µs duration; time range within [1669365093932627; 1669365093962241]
[2022-11-25 09:31:33] Received: EventStore containing 3109 events within 27834µs duration; time range within [1669365093962241; 1669365093990075]
[2022-11-25 09:31:33] Received: EventStore containing 3107 events within 27601µs duration; time range within [1669365093990075; 1669365094017676]
```

The executable will print packet information until a interrupt signal is sent by the user.

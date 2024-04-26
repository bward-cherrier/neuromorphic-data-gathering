# Network streaming

The dv-processing library an easy to integrate solution to stream data over network TCP connections with optional
encryption for security or through local sockets. This achieved by using client-server paradigm implemented using two
classes: {cpp:class}`dv::io::NetworkWriter` and {cpp:class}`dv::io::NetworkReader`. The
{cpp:class}`dv::io::NetworkWriter` acts as a server, supports multiple concurrent clients, and streams a single data
type stream to the clients. The {cpp:class}`dv::io::NetworkReader` connects to a server and is able to receive the
streamed data.

This tutorial will introduce basic usage of these classes to stream event data using a network writer and receive the
same data using network reader.

## Network streaming server

The following sample implements a network streaming server using {cpp:class}`dv::io::NetworkWriter`, that streams
periodic event data, that is synthesized in software:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/network_streaming/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/network_streaming/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

Compiling and running the sample above will run an event streaming server, which is accessible on port 10101 on all
available network devices. The instance will wait for a client to connect, next chapter will introduce a minimal client
application to receive and visualize the streamed data.

## Network streaming client

The following code sample provide a basic usage of {cpp:class}`dv::io::NetworkReader` to receive network streamed data.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/network_streaming/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/network_streaming/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

This code sample connects to the port 10101 on local loopback network, receives event data stream and uses
{cpp:class}`dv::visualization::EventVisualizer` to generate a preview image of the event data.

## Running the samples

We can now combine both samples to stream the synthetic events and visualize the output. First, run the server sample
application, when it prints "Waiting for connections...", launch the second client application, it should show the
preview window with a small rectangle moving from corner to corner.

```{figure} assets/network_streaming/moving_rectangle.png
---
align: center
---
Preview of the rectangle that is received as event packets, the triangle is expected to move diagonally through the
window pixel space.
```

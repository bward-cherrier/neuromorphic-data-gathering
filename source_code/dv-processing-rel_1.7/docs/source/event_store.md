# Event store

The dv-processing library provides an easy to use and an efficient data structure to store and manage incoming event
data from the camera - the {cpp:type}`dv::EventStore` class. It is implemented as a shallow, shared ownership data
structure which only holds pointers to actual memory locations containing event data. Similarly to how OpenCV handles
image data, the {cpp:type}`dv::EventStore` does not copy the data, instead it only holds pointers to received packets of
events and uses metadata of these packets to efficiently slice the data given time-intervals.

## Create and store events

The following sample code shows how to create an empty event store and fill it with software generated events:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_store/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_store/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

## Slicing Event Stores

{cpp:type}`dv::EventStore` can be sliced by time or by the number of events. Slicing is a shallow operation, it does not
copy any data. Slicing returns a new {cpp:type}`dv::EventStore` that only references the data requested. The original
store is unaffected.

### Slicing by time

{cpp:type}`dv::EventStore` implements event data slicing by time in an efficient way. By reusing the underlying packet
structure, the slicing is performed with `O(log n)` time complexity. The following sample shows the usage of time based
slicing functions:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_store/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_store/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Slicing by number of events

{cpp:type}`dv::EventStore` provides slicing capabilities by an index and number of events. The first argument to the
{cpp:func}`dv::EventStore::slice()` method is a starting index from which the output slice starts, the second optional
argument is the number of events to be sliced. If the number argument is not provided, the slice will contain all events
from given index.

Following sample shows how to slice an event store within given indices of the underlying events:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_store/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_store/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Combining multiple EventStores

Multiple instances of {cpp:type}`dv::EventStore` can be added together to have a single object to access and manage it.
Since {cpp:type}`dv::EventStore` uses pointers to underlying event packet data, combining does not involve any deep data
copies, the implementation takes over shared memory ownership instead. Following sample show how to efficiently combine
multiple event stores into one:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_store/sample3.cpp
:language: c++
:linenos:
:tab-width: 4
```
````
````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_store/sample3.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

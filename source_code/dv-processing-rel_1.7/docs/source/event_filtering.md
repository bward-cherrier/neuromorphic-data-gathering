# Filtering events

Event stream coming from a camera can contain noise, which is caused by analog electrical circuitry used to compare
brightness on each pixel. The dv-processing library provides algorithms that filter noise efficiently on an event
stream. Additionally, filtering can be used to subsample events in region of interest, filter by polarity, apply masks
to events. This tutorial covers available filter implementations and how to use them efficiently.

## Implementation of filters

The library provides two main types of filters - noise and subsampling. Subsampling filters include polarity filters,
region of interest, mask filters. The library provides two algorithms for filtering noise that can be found under
namespace `dv::noise`. Below is a class hierarchy diagram for available noise filters:

```{figure} assets/event_filtering/filter_class_diagram.png
---
align: center
---
Available filters and their hierarchy in the library.
```

All filter have a common programming pattern:

- Events are added to the filter instance using overloaded {cpp:func}`dv::EventFilterBase::accept()` method.
- Input events are filtered and returned by calling overloaded {cpp:func}`dv::EventFilterBase::generateEvents()` method.
- Internally events are filtered by calling a filter `retain()` method on each event and discarding events if a `false`
  is returned.

## Noise filtering

This chapter describes the available event noise filters in the library.

### Background activity noise filter

{cpp:class}`dv::noise::BackgroundActivityNoiseFilter` - events are filtered based on short-term local neighborhood
activity. If an event is "supported" by another event registered at local pixel neighborhood, that event is not
considered noise and is added to the output events.

Following sample code shows the usage of the {cpp:class}`dv::noise::BackgroundActivityNoiseFilter` to filter noise:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/bg_activity_filter.jpg
---
align: center
---
Output of the sample use of background activity filter. Left is a preview of input events, right is a preview
of filtered events.
```

```{note}
The image above that the filter reduces the amount of speckles on white area, but maintains the logo preview,
those events are not filtered out.
```

### Fast decay noise filter

{cpp:class}`dv::noise::FastDecayNoiseFilter` - events are filtered based on lower resolution fast-decaying
representation of events. Events contribute to a low-resolution accumulated image with a fast decay, which also
represents local activity. Unlike the `BackgroundActivityNoiseFilter`, this filter uses decay instead of a hard time
threshold, although the approach is very similar - an event needs to be supported by another event in a local pixel
neighborhood. This filter has a lower memory footprint since the neighborhood is represented in a low resolution
accumulated image.

Following sample code shows the usage of the {cpp:class}`dv::noise::FastDecayNoiseFilter` to filter noise:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample1.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample1.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/fast_decay_filter.png
---
align: center
---
Output of the sample use of fast decay noise filter. Left is a preview of input events, right is a preview
of filtered events.
```

```{note}
The image above that the filter drastically the amount of speckles on white area, although the logo image is
also affected and some true-signal events contributing to the logo are also filtered out.
```

## Event subsampling

The same filtering approach is used to subsampling events based on their pixel location, polarity or other properties.
This chapter describes the available event subsampling filters in the library.

### Mask filter

{cpp:class}`dv::EventMaskFilter` - filters events based on a pixel mask. Events are discarded in pixel locations where
mask has zero values.

Following sample code shows the usage of the {cpp:class}`dv::EventMaskFilter` to filter out selected regions of events:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample2.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample2.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/mask_filter.png
---
align: center
---
Output of the sample use of event mask filter. Left is a preview of input events, middle is the mask used,
and right is a preview of filtered events.
```

### Refractory period filter

{cpp:class}`dv::RefractoryPeriodFilter` - refractory period filter discards bursts of events at repeating pixel
locations. Each event timestamp is compared against most recent event timestamp on the same pixel location, if the
timestamp difference is less than the refractory period, the event is discarded.

Following sample code shows the usage of the {cpp:class}`dv::RefractoryPeriodFilter` to filter out events that are
within close time period on the same coordinate location:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample3.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample3.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

### Polarity filter

{cpp:class}`dv::EventPolarityFilter` - filter events based on polarity.

Following sample code shows the usage of the {cpp:class}`dv::EventPolarityFilter` to filter out events based on
polarity:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample4.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample4.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/polarity_filter.png
---
align: center
---
Output of the sample use of polarity filter. Left is a preview of input events
and right is a preview of filtered events.
```

```{note}
The generated DV logo image comes from event representation, white area means no events are there, dark grey
area are negative events and pixels are blue on coordinates where positive polarity events are provided. The filtered
image only contains the letters "DV" since the background circle is represented by negative polarity events.
```

### Event region filter

{cpp:class}`dv::EventRegionFilter` - filter events based on given region of interest.

Following sample code shows the usage of the {cpp:class}`dv::EventRegionFilter` to filter out specific area of events:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample5.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample5.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/roi_filter.png
---
align: center
---
Output of the sample use of region filter. Left is a preview of input events
and right is a preview of filtered events.
```

### Filter chain

Multiple filters can be combined into a single filter chain, which optimizes memory operations to increase the
performance of applying multiple filters. This is achieved by using the {cpp:class}`dv::EventFilterChain` class.
Multiple filters can be added using {cpp:func}`dv::EventFilterChain::addFilter()` method, it accepts filter wrapped in
`std::shared_pointer`, the shared pointer is used to be able to modify the parameters of filters after they are added to
the filter chain.

Following sample code shows the usage of the {cpp:class}`dv::EventFilterChain` to apply multiple types of filters in a
single chain:

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/event_filtering/sample6.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/event_filtering/sample6.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

The sample code outputs such images:

```{figure} assets/event_filtering/filter_chain.png
---
align: center
---
Output of the sample use of multiple filters in a filter chain. Left is a preview of input events
and right is a preview of filtered events.
```

## Filtering performance

The provided event filters performance is measured using benchmarks, the benchmarks for filters can be found under
directory `benchmarks/noise` in the project repository. These are sample benchmarking results, performance of filters is
measured in throughput of mega-events per second. These measurements were capture on an AMD Ryzen 5 3600 6-Core
processor.

| Filter                        | Event count per iteration | Throughput, MegaEvents / second |
| ----------------------------- | ------------------------- | ------------------------------- |
| FastDecayNoiseFilter          | 1000                      | 82.3                            |
| FastDecayNoiseFilter          | 4096                      | 82.9                            |
| FastDecayNoiseFilter          | 32768                     | 80.1                            |
| FastDecayNoiseFilter          | 262144                    | 67.5                            |
| FastDecayNoiseFilter          | 1000000                   | 67.2                            |
| BackgroundActivityNoiseFilter | 1000                      | 141.5                           |
| BackgroundActivityNoiseFilter | 4096                      | 139.5                           |
| BackgroundActivityNoiseFilter | 32768                     | 105.4                           |
| BackgroundActivityNoiseFilter | 262144                    | 134.6                           |
| BackgroundActivityNoiseFilter | 1000000                   | 135.0                           |
| RefractoryPeriodFilter        | 1000                      | 268.7                           |
| RefractoryPeriodFilter        | 4096                      | 278.9                           |
| RefractoryPeriodFilter        | 32768                     | 254.9                           |
| RefractoryPeriodFilter        | 262144                    | 255.1                           |
| RefractoryPeriodFilter        | 1000000                   | 167.5                           |
| PolarityFilter                | 1000                      | 503.7                           |
| PolarityFilter                | 4096                      | 345.0                           |
| PolarityFilter                | 32768                     | 165.4                           |
| PolarityFilter                | 262144                    | 157.7                           |
| PolarityFilter                | 1000000                   | 156.6                           |
| RegionFilter                  | 1000                      | 446.9                           |
| RegionFilter                  | 4096                      | 371.7                           |
| RegionFilter                  | 32768                     | 166.8                           |
| RegionFilter                  | 262144                    | 153.6                           |
| RegionFilter                  | 1000000                   | 151.0                           |
| MaskFilter                    | 1000                      | 421.5                           |
| MaskFilter                    | 4096                      | 264.8                           |
| MaskFilter                    | 32768                     | 123.6                           |
| MaskFilter                    | 262144                    | 118.0                           |
| MaskFilter                    | 1000000                   | 117.1                           |
| ThreeFiltersNoChain           | 1000                      | 89.3                            |
| ThreeFiltersNoChain           | 4096                      | 88.0                            |
| ThreeFiltersNoChain           | 32768                     | 76.2                            |
| ThreeFiltersNoChain           | 262144                    | 75.0                            |
| ThreeFiltersNoChain           | 1000000                   | 72.6                            |
| ThreeFiltersWithChain         | 1000                      | 170.4                           |
| ThreeFiltersWithChain         | 4096                      | 99.1                            |
| ThreeFiltersWithChain         | 32768                     | 69.4                            |
| ThreeFiltersWithChain         | 262144                    | 70.3                            |
| ThreeFiltersWithChain         | 1000000                   | 70.1                            |

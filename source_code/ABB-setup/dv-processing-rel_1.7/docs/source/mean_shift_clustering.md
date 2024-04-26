# Mean-shift clustering

Mean-shift clustering algorithm is used to find high density clusters of events (or point) within a set of events. The
dv-processing library provides an implementation of such an algorithm which is efficiently implemented for incoming
streams of events from event cameras.

The algorithm can be summarized as follows:

1. Given a set of coordinates, for each coordinate retrieve a neighborhood of events within a configured window.
1. Calculate the mean of coordinates for the retrieved neighborhood.
1. Shift the initial coordinates by a mode, which is a vector going from the initial point to the mean coordinate
   multiplied by a learning rate factor.
1. If the mode of a vector is lower than a configured threshold, the coordinate is considered to have converged into a
   cluster center, otherwise repeat from step one.

```{figure} assets/mean_shift_clustering/mean_shift_clustering.gif
---
align: center
---
An approximate visualization of a mean-shift clustering algorithm to find centers of coordinate clusters in a 2D plane.
```

This algorithm is useful to find event clusters that could be used as point of interest in event processing algorithms.

## Sample usage

The following code sample shows the use of mean-shift clustering implementation to find and track clusters of events on
simulated events.

`````{tabs}
````{group-tab} C++
```{literalinclude} assets/code_samples/c++/mean_shift_clustering/sample0.cpp
:language: c++
:linenos:
:tab-width: 4
```
````

````{group-tab} Python
```{literalinclude} assets/code_samples/python/mean_shift_clustering/sample0.py
:language: python
:linenos:
:tab-width: 4
```
````
`````

```{figure} assets/mean_shift_clustering/clustering_preview.png
---
align: center
---
Expected output of the mean-shift sample usage, four cluster centers marked with red crosses.
```

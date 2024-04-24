# Multi stream slicing

This sample provides an example usage of a `dv::MultiStreamSlicer` class. It can be used to slice multiple streams
containing time-series data into time-synchronized batches. This sample is compatible with DVXplorer and DAVIS cameras,
when running it, it will open the camera, read the data, synchronize multiple stream of data (events, frame, etc.) and
show a time synchronized preview of the data.

The slicer can be used on arbitrary number of streams, coming from different sources, as long as the timestamps of data
stream elements are synchronized. Inter-camera synchronization is required for multi-camera setups.

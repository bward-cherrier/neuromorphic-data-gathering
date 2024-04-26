import dv_processing as dv

# Sample VGA resolution, same as the DVXplorer camera
resolution = (640, 480)

# Event only configuration
config = dv.io.MonoCameraWriter.EventOnlyConfig("DVXplorer_sample", resolution)

# Create the writer instance, it will only have a single event output stream.
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

# Write 100 packet of event data
for i in range(100):
    # EventStore requires strictly monotonically increasing data, generate
    # a timestamp from the iteration counter value
    timestamp = i * 1000

    # Empty event store
    events = dv.data.generate.dvLogoAsEvents(timestamp, resolution)

    # Write the packet using the writer, the data is not going be written at the exact
    # time of the call to this function, it is only guaranteed to be written after
    # the writer instance is destroyed (destructor has completed)
    writer.writeEvents(events)

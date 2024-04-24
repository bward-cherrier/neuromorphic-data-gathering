import dv_processing as dv
from datetime import timedelta

# Initialize slicer, it will have no jobs at this time
slicer = dv.EventStreamSlicer()


def print_time_interval(events: dv.EventStore):
    # Print the time duration received by this method
    print(f"* Received events time-based slicing: {events}")


# Register this method to be called every 33 millisecond worth of event data
slicer.doEveryTimeInterval(timedelta(milliseconds=33), print_time_interval)


def print_event_number(events: dv.EventStore):
    # Print the number of events received here
    print(f"# Received events in number-based slicing: {events}")


# Register this method to be called every 100 events
slicer.doEveryNumberOfEvents(100, print_event_number)

# Generate 1000 events within 2 second interval. These will be sliced correctly by the slicer.
store = dv.data.generate.uniformEventsWithinTimeRange(0, timedelta(seconds=2), (100, 100), 1000)

# Now push the store into the slicer, the data contents within the store
# can be arbitrary, the slicer implementation takes care of correct slicing
# algorithm and calls the previously registered callbacks accordingly.
slicer.accept(store)

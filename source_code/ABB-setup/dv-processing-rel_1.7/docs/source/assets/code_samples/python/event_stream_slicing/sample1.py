import dv_processing as dv
from datetime import timedelta

# Initialize slicer, it will have no jobs at this time
slicer = dv.EventStreamSlicer()


def print_time_interval(events: dv.EventStore):
    # Print the time duration received by this method
    print(f"* Received event with {events.duration()} duration in time-based slicing")


# Register this method to be called every 33 millisecond worth of event data
time_job_id = slicer.doEveryTimeInterval(timedelta(milliseconds=33), print_time_interval)


def print_event_number(events: dv.EventStore):
    # Print the number of events received here
    print(f"# Received {events.size()} events in number-based slicing")


# Register this method to be called every 100 events
number_job_id = slicer.doEveryNumberOfEvents(100, print_event_number)

# Implement data generation; The following loop will generate 10 stores
# of events, each containing 100 events within 10 millisecond duration.
for i in range(10):
    # Generate 100 events within 20 millisecond interval. These will be sliced correctly by the slicer.
    store = dv.data.generate.uniformEventsWithinTimeRange(i * 20000, timedelta(milliseconds=20), (100, 100), 100)

    # Now push the store into the slicer, the data contents within the store
    # can be arbitrary, the slicer implementation takes care of correct slicing
    # algorithm and calls the previously registered callbacks accordingly.
    slicer.accept(store)

    # When a packet with index 5 is reached, modify the parameters
    if i == 5:
        # Modify time range to 10 milliseconds instead of 33
        slicer.modifyTimeInterval(time_job_id, timedelta(milliseconds=10))
        # Modify number to 200 instead of 100
        slicer.modifyNumberInterval(number_job_id, 200)

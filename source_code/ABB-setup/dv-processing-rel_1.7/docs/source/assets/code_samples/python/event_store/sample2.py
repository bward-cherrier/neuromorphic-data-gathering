import dv_processing as dv
from datetime import timedelta

# Generate 10 events with time range [10000; 20000]
store = dv.data.generate.uniformEventsWithinTimeRange(10000, timedelta(milliseconds=10), (100, 100), 10)

# Get all events beyond and including index 5
events_after_index = store.slice(5)
print(f"1. {events_after_index}")

# Get 3 events starting with index 2
events_in_range = store.slice(2, 3)
print(f"2. {events_in_range}")

# Use sliceBack to retrieve event from the end; this call will retrieve last 3 events
last_events = store.sliceBack(3)
print(f"3. {last_events}")

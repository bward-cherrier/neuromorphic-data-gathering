import dv_processing as dv
from datetime import timedelta

# Generate 10 events with time range [10000; 20000]
store = dv.data.generate.uniformEventsWithinTimeRange(10000, timedelta(milliseconds=10), (100, 100), 10)

# Get all events with timestamp above 12500, it will be 13000 and up
eventsAfterTimestamp = store.sliceTime(12500)

# Print the timestamp ranges
print(f"1. {eventsAfterTimestamp}")

# Slice event within time range [12000; 16000); the end time is exclusive
eventsInRange = store.sliceTime(12000, 16000)

# Print the timestamp ranges; It will print that range is [12000; 15000] since end time is exclusive and
# event at timestamp 16000 is not going to be included.
print(f"2. {eventsInRange}")

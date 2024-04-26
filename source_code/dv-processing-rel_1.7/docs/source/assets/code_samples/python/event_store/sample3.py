import dv_processing as dv
from datetime import timedelta

# Generate 10 events with timestamps in range [10000; 20000]
store1 = dv.data.generate.uniformEventsWithinTimeRange(10000, timedelta(milliseconds=10), (100, 100), 10)

# Generate second event store with 10 events with timestamps in range [20000; 29000] to the second store
store2 = dv.data.generate.uniformEventsWithinTimeRange(20000, timedelta(milliseconds=10), (100, 100), 10)

# Final event store which will contain all events
final_store = dv.EventStore()

# Add the events into the final store; this operation is shallow, so no data copies
# are performed, but the underlying data has shared ownership between all stores
final_store.add(store1)
final_store.add(store2)

# Print specific information on what we contain in the final event store
print(f"{final_store}")

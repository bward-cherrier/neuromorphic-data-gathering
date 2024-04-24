import dv_processing as dv

# Initialize an empty store
store = dv.EventStore()

# Get the current timestamp
timestamp = dv.now()

# Add some events into the event store
# This allocates and inserts events at the back, the function arguments are:
# timestamp, x, y, polarity
store.push_back(timestamp, 0, 0, True)
store.push_back(timestamp + 1000, 1, 1, False)
store.push_back(timestamp + 2000, 2, 2, False)
store.push_back(timestamp + 3000, 3, 3, True)

# Perform time-based slicing of event store, the output event store "sliced" will contain
# the second and third events from above. The end timestamp (second argument) is 2001, since start
# timestamp (first argument) is inclusive and timestamp is exclusive, so 1 is added.
sliced = store.sliceTime(timestamp + 1000, timestamp + 2001)

# This should print two events
for ev in store:
    print(f"Sliced event [{ev.timestamp()}, {ev.x()}, {ev.y()}, {ev.polarity()}]")

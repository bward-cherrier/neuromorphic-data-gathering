import dv_processing as dv
from datetime import timedelta
import time

# Define image space resolution dimensions
resolution = (200, 200)

# Define output event stream with valid resolution
stream = dv.io.Stream.EventStream(0, "events", "TEST_DATA", resolution)

# Initiate the server, needs stream definition to initiate
server = dv.io.NetworkWriter("0.0.0.0", 10101, stream)

# Print the ready state of the server
print("Waiting for connections...")

# Stream interval defines the packet frequency for this sample
streamInterval = timedelta(milliseconds=10)

# Starting coordinates of the rectangle data that is going to be sent out in this sample
offset = (0, 0)

# Rectangle size in pixels
rectSize = (20, 20)

# A boolean variable used to define movement direction of the rectangle
direction = True

# Run indefinitely
while True:
    # Do not produce output if there are no connected clients
    if server.getClientCount() > 0:
        # Generate the rectangle at given offset position
        events = dv.data.generate.eventRectangle(dv.now(), offset, (offset[0] + rectSize[0], offset[1] + rectSize[1]))

        # Increase or decrease the position coordinates depending on the "direction" boolean
        offset = (offset[0] + 1, offset[1] + 1) if direction else (offset[0] - 1, offset[1] - 1)

        # Check if the rectangle coordinates reaches borders of the image
        if offset[0] == 0 or offset[1] == 0 or offset[0] + rectSize[0] == resolution[0] or offset[1] + rectSize[1] == \
                resolution[1]:
            # Reverse the motion direction
            direction = not direction

        # Send it out to clients
        server.writeEvents(events)

    # Sleep the application for the streaming interval duration
    time.sleep(streamInterval.total_seconds())

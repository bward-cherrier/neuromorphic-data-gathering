import dv_processing as dv
import numpy as np

# Frame only configuration
config = dv.io.MonoCameraWriter.FrameOnlyConfig("DVXplorer_sample", (640, 480))

# Create the writer instance, it will only have a single frame output stream
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

# Write 10 image frames
for i in range(10):
    # Initialize a white image
    image = np.full((480, 640, 3), fill_value=255, dtype=np.uint8)

    # Generate some monotonically increasing timestamp
    timestamp = i * 1000

    # Encapsulate the image in a frame that has a timestamp, this does not copy the pixel data
    frame = dv.Frame(timestamp, image)

    # Write the frame
    writer.writeFrame(frame)

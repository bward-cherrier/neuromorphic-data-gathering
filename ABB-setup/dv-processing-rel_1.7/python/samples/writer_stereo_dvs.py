import dv_processing as dv
import argparse

parser = argparse.ArgumentParser(
    description='Save a stereo event stream from a pair of time-synchronized iniVation cameras to a file.')

args = parser.parse_args()

# Open the cameras, provide exact names for left and right cameras
camera = dv.io.StereoCapture("DVXplorer_DXA00080", "DAVIS346_00000600")

# Try block to catch user interrupt to stop recording
try:
    # Open a file, pass camera instance for the writer to inspect what data streams have to be recorded
    writer = dv.io.StereoCameraWriter("./stereo.aedat4", camera)

    print("Start recording")
    # While both cameras are physically present
    while camera.left.isConnected() and camera.right.isConnected():
        # Read a packet from the camera
        leftEvents = camera.left.getNextEventBatch()
        # If read was successful
        if leftEvents is not None:
            # Write the packet into the file
            writer.left.writeEvents(leftEvents)

        # Equivalent execution for the right camera
        rightEvents = camera.right.getNextEventBatch()
        if rightEvents is not None:
            writer.right.writeEvents(rightEvents)
except KeyboardInterrupt:
    pass

print("Ending recording")

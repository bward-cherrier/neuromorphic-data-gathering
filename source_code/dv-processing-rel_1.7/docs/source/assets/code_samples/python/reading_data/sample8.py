import dv_processing as dv

# Open a file
reader = dv.io.MonoCameraRecording("path/to/file.aedat4")

# Get and print the camera name that data from recorded from
print(f"Opened an AEDAT4 file which contains data from [{reader.getCameraName()}] camera")

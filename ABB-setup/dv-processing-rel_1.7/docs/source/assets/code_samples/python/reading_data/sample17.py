import dv_processing as dv

# Open a Davis camera
capture = dv.io.CameraCapture(dv.io.CameraCapture.CameraType.DAVIS)

# Setting camera readout to events and frames (default). Other modes available: EventsOnly, FramesOnly
capture.setDavisReadoutMode(dv.io.CameraCapture.DavisReadoutMode.EventsAndFrames)
# Configure frame output mode to color (default), only on COLOR cameras. Other mode available: Grayscale
capture.setDavisColorMode(dv.io.CameraCapture.DavisColorMode.Color)

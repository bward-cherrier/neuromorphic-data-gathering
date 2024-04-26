import dv_processing as dv

# Open a Davis camera
capture = dv.io.CameraCapture(dv.io.CameraCapture.CameraType.DAVIS)

# Enable frame auto-exposure (default behavior)
capture.enableDavisAutoExposure()
# Disable auto-exposure, set frame exposure (here 10ms)
capture.setDavisExposureDuration(10000)
# Read current frame exposure duration value
duration = capture.getDavisExposureDuration()
# Set frame interval duration (here 33ms for ~30FPS)
capture.setDavisFrameInterval(33000)
# Read current frame interval duration value
interval = capture.getDavisFrameInterval()

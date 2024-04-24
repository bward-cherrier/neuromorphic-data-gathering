import dv_processing as dv

# Open a DVXplorer camera
capture = dv.io.CameraCapture(dv.io.CameraCapture.CameraType.DVS)

# Configure default event sensitivity. Other sensitivities available: VeryLow, Low, High, VeryHigh
capture.setDVSBiasSensitivity(dv.io.CameraCapture.BiasSensitivity.Default)

# Configure event-frame readouts per second (here variable 5000 FPS, the default value)
# See detailed API documentation for other available values
capture.setDVXplorerEFPS(dv.io.CameraCapture.DVXeFPS.EFPS_VARIABLE_5000)

# Disable global hold setting. Default is True
capture.setDVSGlobalHold(False)
# Enable global reset setting. Default is False
capture.setDVXplorerGlobalReset(True)

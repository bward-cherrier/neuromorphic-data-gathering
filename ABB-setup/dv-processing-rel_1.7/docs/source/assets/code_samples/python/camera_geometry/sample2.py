import dv_processing as dv
import cv2 as cv
import numpy as np

# Use VGA resolution for this sample
resolution = (640, 480)

# Initialize an ideal pinhole camera model parameters with no distortion
geometry = dv.camera.CameraGeometry(640, 640, 320, 240, resolution)

# Generate a sample set of events and filter out only positive events for this sample
positiveEvents = dv.polarityFilter(dv.data.generate.dvLogoAsEvents(0, resolution), True)

# Back project the events into a set of 3D points
points = geometry.backProjectSequence(positiveEvents)

# Apply some 3D transformation
shift = dv.kinematics.Transformationf(0, (0.5, 0.3, 0.1), (0.24, -0.31, -0.89, 0.18))

# Apply the transformation above to each of the back-projected points
shiftedPoints = []
for point in points:
    shiftedPoints.append(shift.transformPoint(point))

# Forward project the points with transformation
rotatedPixels = geometry.projectSequence(shiftedPoints)

# Draw input events on an image for input preview
input = np.ndarray((480, 640, 3), dtype=np.uint8)
input.fill(255)
for event in positiveEvents:
    input[event.y(), event.x(), :] = dv.visualization.colors.iniBlue()

# Draw output pixels on another image
output = np.ndarray((480, 640, 3), dtype=np.uint8)
output.fill(255)
for pixel in rotatedPixels:
    output[int(pixel[1]), int(pixel[0]), :] = dv.visualization.colors.iniBlue()

# Create a window and show a concatenated preview image of input events and output coordinates
cv.namedWindow("Preview", cv.WINDOW_NORMAL)
cv.imshow("Preview", cv.hconcat([input, output]))
cv.waitKey()

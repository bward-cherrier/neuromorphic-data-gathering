import dv_processing as dv
import numpy as np

# Mirror rotation matrix with 0.5 translational offsets on all axes. The rotation matrix should flip
# x and z axes of the input.
matrix = np.array([[-1.0, 0.0, 0.0, 0.5], [0.0, 1.0, 0.0, 0.5], [0.0, 0.0, -1.0, 0.5], [0.0, 0.0, 0.0, 1.]])

# Initialize the transformation with the above matrix. The timestamp can be ignored for this sample, so its set
# to zero.
transformation = dv.kinematics.Transformationf(0, matrix)

# Let's take a sample point with offsets of 1 on all axes.
point = np.array([1.0, 1.0, 1.0])

# Apply this transformation to the above point. This should invert x and z axes and add 0.5 to all values.
transformed = transformation.transformPoint(point)

# Print the resulting output.
print(f"Transformed from {point} to {transformed}")

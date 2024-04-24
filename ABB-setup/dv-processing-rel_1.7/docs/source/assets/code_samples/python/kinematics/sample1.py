import dv_processing as dv
import numpy as np

# Declare linear transformer with capacity of 100 transformations. Internally it uses a bounded FIFO queue
# to manage the transformations.
transformer = dv.kinematics.LinearTransformerf(100)

# Push first transformation which is an identity matrix, so it starts with no rotation at zero coordinates
transformer.pushTransformation(dv.kinematics.Transformationf(1000000, np.array([0.0, 0.0, 0.0]), (1.0, 0.0, 0.0, 0.0)))

# Add a second transformation with no rotation as well, but with different translational coordinates
transformer.pushTransformation(dv.kinematics.Transformationf(2000000, np.array([1.0, 2.0, 3.0]), (1.0, 0.0, 0.0, 0.0)))

# Interpolate transformation at a midpoint (time-wise), this should device the translational coordinates
# by a factor of 2.0
midpoint = transformer.getTransformAt(1500000)

# Print the resulting output.
print(f"Interpolated position at [{midpoint.getTimestamp()}]: {midpoint.getTranslation()}")

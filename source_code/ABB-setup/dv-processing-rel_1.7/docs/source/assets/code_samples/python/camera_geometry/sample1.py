import dv_processing as dv

# Initialize a calibration set
calibration_set = dv.camera.CalibrationSet.LoadFromFile("calibration.json")

# Iterate through available camera calibrations. The designation here is an internal camera abbreviation
# used to refer to a specific sensor in the camera rig.
for designation, calibration in calibration_set.getCameraCalibrations().items():
    # Print the designation and the camera name of current calibration
    print(f"[{designation}] Found calibration for camera with name [{calibration.name}]")

    # Print the intrinsic calibration parameters for this camera: focal length, principal point, distortion model
    # and parameters of the distortion model
    print(f"\t Focal length: {calibration.focalLength}")
    print(f"\t Principal point: {calibration.principalPoint}")
    print(f"\t Distortion model: {calibration.distortionModel}")
    print(f"\t Distortion parameters: {calibration.distortion}")

# Iterate through available IMU calibrations in the file
for designation, calibration in calibration_set.getImuCalibrations().items():
    # Print the designation and the camera name of current calibration
    print(f"[{designation}] Found IMU calibration for camera with name [{calibration.name}]")

    # Print some available information: accelerometer and gyroscope measurement limits, calibrated time offset and
    # biases
    print(f"\t Maximum acceleration: {calibration.accMax} [m/s^2]")
    print(f"\t Maximum angular velocity: {calibration.omegaMax} [rad/s]")
    print(f"\t Time offset: {calibration.timeOffsetMicros} [μs]")
    print(f"\t Accelerometer bias: {calibration.accOffsetAvg} [m/s^2]")
    print(f"\t Gyroscope bias: {calibration.omegaOffsetAvg} [rad/s]")

    # Print noise density values for the IMU sensor
    print(f"\t Accelerometer noise density: {calibration.accNoiseDensity} [m/s^2/sqrt(Hz)]")
    print(f"\t Gyroscope noise density: {calibration.omegaNoiseDensity} [rad/s/sqrt(Hz)]")

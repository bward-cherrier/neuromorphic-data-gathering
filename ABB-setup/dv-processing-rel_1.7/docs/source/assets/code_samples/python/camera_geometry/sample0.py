import dv_processing as dv

calibration = dv.camera.CalibrationSet()

calibration.addCameraCalibration(
    dv.camera.calibrations.CameraCalibration(
        "DVXplorer_DXA000312", "left", True, (640, 480), (320, 240), (640, 640), [], dv.camera.DistortionModel.NONE,
        [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        dv.camera.calibrations.CameraCalibration.Metadata()))

calibration.addImuCalibration(
    dv.camera.calibrations.IMUCalibration(
        "DVXplorer_DXA000312", 100.0, 98.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 3500,
        [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        dv.camera.calibrations.IMUCalibration.Metadata()))

calibration.writeToFile("calibration.json")

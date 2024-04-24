import dv_processing as dv
import pathlib
import unittest


class CalibrationTests(unittest.TestCase):

    def test_load(self):
        filepath = "../../tests/camera/calibration_files/calibration_camera_DVXplorer_DXA00080-2021_11_01_14_32_23.xml"
        if not pathlib.Path(filepath).exists():
            return

        calibration = dv.camera.CalibrationSet.LoadFromFile(filepath)
        self.assertTrue(calibration is not None)
        intrinsics = calibration.getCameraCalibrationByName("DVXplorer_DXA00080")
        self.assertTrue(intrinsics is not None)
        self.assertEqual(intrinsics.name, "DVXplorer_DXA00080")

        geometry = intrinsics.getCameraGeometry()
        self.assertAlmostEqual(geometry.getFocalLength()[0], 698.46246, places=5)
        self.assertAlmostEqual(geometry.getFocalLength()[1], 697.59271, places=5)

    def test_write(self):
        calibration = dv.camera.CalibrationSet()
        intrinsics = dv.camera.calibrations.CameraCalibration("TEST_SERIAL", "left", True, (100, 100), (0.5, 0.5),
                                                              (0.5, 0.5), [1, 2, 3, 4, 5],
                                                              dv.camera.DistortionModel.RadTan,
                                                              [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], None)
        calibration.addCameraCalibration(intrinsics)
        filepath = dv.Path("test-calibration.json")
        calibration.writeToFile(filepath)

        calibration = dv.camera.CalibrationSet.LoadFromFile(filepath)
        self.assertTrue(calibration is not None)
        self.assertTrue(calibration.getCameraCalibrationByName("TEST_SERIAL") is not None)
        self.assertTrue(calibration.getCameraCalibration("C0") is not None)
        self.assertEqual(calibration.getCameraCalibration("C0").name, "TEST_SERIAL")


if __name__ == '__main__':
    unittest.main()

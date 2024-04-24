#include <dv-processing/camera/calibration_set.hpp>

int main() {
	// Initialize a calibration set
	dv::camera::CalibrationSet calibration;

	// Add a camera calibration with hardcoded calibration parameters, the exact values are just for illustration.
	calibration.addCameraCalibration(dv::camera::calibrations::CameraCalibration("DVXplorer_DXA000312", "left", true,
		cv::Size(640, 480), cv::Point2f(320, 240), cv::Point2f(640, 640), {}, dv::camera::DistortionModel::None,
		std::vector<float>({1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}),
		dv::camera::calibrations::CameraCalibration::Metadata()));

	// Add an IMU calibration as well, the exact values are just for illustration of use here.
	calibration.addImuCalibration(dv::camera::calibrations::IMUCalibration("DVXplorer_DXA000312", 100.f, 98.1f,
		cv::Point3f(0.f, 0.f, 0.f), cv::Point3f(0.f, 0.f, 0.f), 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 3500,
		std::vector<float>({1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}),
		dv::camera::calibrations::IMUCalibration::Metadata()));

	// Just write the generated calibration set into a file.
	calibration.writeToFile("calibration.json");

	return 0;
}

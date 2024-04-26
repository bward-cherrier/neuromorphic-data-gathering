#include <dv-processing/camera/calibration_set.hpp>

int main() {
	// Initialize a calibration set
	const auto calibrationSet = dv::camera::CalibrationSet::LoadFromFile("calibration.json");

	// Iterate through available camera calibrations. The designation here is an internal camera abbreviation
	// used to refer to a specific sensor in the camera rig.
	for (const auto &[designation, calibration] : calibrationSet.getCameraCalibrations()) {
		// Print the designation and the camera name of current calibration
		std::cout << "[" << designation << "] Found calibration for camera with name [" << calibration.name << "]"
				  << std::endl;

		// Print the intrinsic calibration parameters for this camera: focal length, principal point, distortion model
		// and parameters of the distortion model
		std::cout << "\t Focal length: " << calibration.focalLength << std::endl;
		std::cout << "\t Principal point: " << calibration.principalPoint << std::endl;
		std::cout << "\t Distortion model: " << calibration.getDistortionModelString() << std::endl;
		std::cout << "\t Distortion parameters: "
				  << fmt::format("[{}]", fmt::join(calibration.distortion.begin(), calibration.distortion.end(), ", "))
				  << std::endl;
	}

	// Iterate through available IMU calibrations in the file
	for (const auto &[designation, calibration] : calibrationSet.getImuCalibrations()) {
		// Print the designation and the camera name of current calibration
		std::cout << "[" << designation << "] Found IMU calibration for camera with name [" << calibration.name << "]"
				  << std::endl;

		// Print some available information: accelerometer and gyroscope measurement limits, calibrated time offset and
		// biases
		std::cout << "\t Maximum acceleration: " << calibration.accMax << " [m/s^2]" << std::endl;
		std::cout << "\t Maximum angular velocity: " << calibration.omegaMax << " [rad/s]" << std::endl;
		std::cout << "\t Time offset: " << calibration.timeOffsetMicros << " [μs]" << std::endl;
		std::cout << "\t Accelerometer bias: " << calibration.accOffsetAvg << " [m/s^2]" << std::endl;
		std::cout << "\t Gyroscope bias: " << calibration.omegaOffsetAvg << " [rad/s]" << std::endl;

		// Print noise density values for the IMU sensor
		std::cout << "\t Accelerometer noise density: " << calibration.accNoiseDensity << " [m/s^2/sqrt(Hz)]"
				  << std::endl;
		std::cout << "\t Gyroscope noise density: " << calibration.omegaNoiseDensity << " [rad/s/sqrt(Hz)]"
				  << std::endl;
	}

	return 0;
}

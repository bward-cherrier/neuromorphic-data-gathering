#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/io/camera_capture.hpp>

#include "CLI/CLI.hpp"

#include <Eigen/Core>
#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>

#include <filesystem>
#include <iostream>
#include <limits>

namespace fs = std::filesystem;

int main(int argc, char **argv) {
	// UTF-8 support for CLI arguments on Windows.
	boost::nowide::args utfArgs(argc, argv);

	std::string cameraName;
	float varianceThreshold;
	double duration;
	fs::path calibrationPath;

	CLI::App app{"IMU Bias estimation utility for iniVation cameras"};

	app.add_option("-c,--camera-name", cameraName,
		   "Provide camera name to open. Application will open first discovered camera if a name is not provided.")
		->required();
	app.add_option("-t,--variance-threshold", varianceThreshold,
		   "Maximum variance that can be measured on IMU data. This value is used to determine motion in the data.")
		->default_val(0.1f);
	app.add_option("-d,--duration", duration, "Time duration for collecting sample data in seconds.")->default_val(1.0);
	app.add_option("-a,--calibration-file", calibrationPath, "Path to the calibration file to store the biases values.")
		->check(CLI::ExistingFile);

	try {
		app.parse(argc, argv);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	dv::io::CameraCapture capture(cameraName);

	std::vector<std::vector<float>> accValues(3);
	std::vector<std::vector<float>> gyroValues(3);

	int64_t startTimestamp = -1;
	int64_t endTimestamp   = -1;

	const auto collectionDuration = static_cast<int64_t>(1e+6 * duration);

	boost::nowide::cout << "Opened camera [" << capture.getCameraName() << "]" << std::endl;
	boost::nowide::cout << "Keep the camera steady on a level surface and press ENTER to start data collection ..."
						<< std::endl;
	boost::nowide::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	// Collect the data
	while (const auto batch = capture.getNextImuBatch()) {
		// Skip empty batch
		if (batch->empty()) {
			continue;
		}

		// Convert and push into buffers
		for (const dv::IMU &measurement : *batch) {
			const auto accelerations = measurement.getAccelerations();
			accValues[0].push_back(accelerations[0]);
			accValues[1].push_back(accelerations[1]);
			accValues[2].push_back(accelerations[2]);

			const auto velocities = measurement.getAngularVelocities();
			gyroValues[0].push_back(velocities[0]);
			gyroValues[1].push_back(velocities[1]);
			gyroValues[2].push_back(velocities[2]);
		}

		// Stop when enough data is collected
		if (startTimestamp < 0) {
			startTimestamp = batch->front().timestamp;
		}

		endTimestamp = batch->back().timestamp;

		if ((endTimestamp - startTimestamp) > collectionDuration) {
			break;
		}
	}

	// Print some info
	const size_t sampleSize = accValues[0].size();
	boost::nowide::cout << fmt::format(
		"Collected {} samples over {} seconds", sampleSize, static_cast<double>(endTimestamp - startTimestamp) * 1e-6)
						<< std::endl;

	// Validate standard deviation to make sure the data device was stable during
	// collection
	std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> values;

	// Mapping the data
	for (const auto &acc : accValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(acc.data(), static_cast<Eigen::Index>(acc.size()));
		values.push_back(data);
	}

	for (const auto &gyro : gyroValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(gyro.data(), static_cast<Eigen::Index>(gyro.size()));
		values.push_back(data);
	}

	std::vector<float> biases;

	// Sanity checking and mean estimation.
	for (const auto &sequence : values) {
		const float mean = sequence.mean();
		const float variance
			= std::sqrt((sequence.array() - mean).square().sum() / static_cast<float>(sequence.size() - 1));

		if (variance > varianceThreshold) {
			throw dv::exceptions::RuntimeError("Some motion detected in IMU data, please keep the device steady "
											   "while collecting and repeat.");
		}

		// Bias at position 1 is Accelerometer Y axis, which is gravity if camera is on a level surface.
		if (biases.size() == 1) {
			static constexpr float earthG = 9.81007f;
			biases.push_back(mean + earthG);
		}
		else {
			biases.push_back(mean);
		}
	}

	// Result printing
	boost::nowide::cout << "Bias estimation successful!" << std::endl;
	boost::nowide::cout << fmt::format("Accelerometer biases [x, y, z] in m/s^2:\n\t[{}, {}, {}]", biases[0], biases[1],
		biases[2]) << std::endl;
	boost::nowide::cout << fmt::format("Gyroscope biases [x, y, z] in rad/s:\n\t[{}, {}, {}]", biases[3], biases[4],
		biases[5]) << std::endl;

	// If calibration file is not provided,
	if (calibrationPath.empty()) {
		return EXIT_SUCCESS;
	}

	auto calibSet = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
	if (auto imuCalib = calibSet.getImuCalibrationByName(capture.getCameraName()); imuCalib.has_value()) {
		imuCalib->accOffsetAvg.x = biases[0];
		imuCalib->accOffsetAvg.y = biases[1];
		imuCalib->accOffsetAvg.z = biases[2];

		imuCalib->omegaOffsetAvg.x = biases[3];
		imuCalib->omegaOffsetAvg.y = biases[4];
		imuCalib->omegaOffsetAvg.z = biases[5];

		calibSet.updateImuCalibration(*imuCalib);
	}
	else {
		std::vector<float> identityMatrix(
			{1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f});
		dv::camera::calibrations::IMUCalibration newImuCalib(capture.getCameraName(), -1.f, -1.f,
			cv::Point3f(biases[3], biases[4], biases[5]), cv::Point3f(biases[0], biases[1], biases[2]), -1.f, -1.f,
			-1.f, -1.f, -1.f, -1.f, 0, identityMatrix, std::nullopt);
		calibSet.addImuCalibration(newImuCalib);
	}
	calibSet.writeToFile(calibrationPath);
	boost::nowide::cout << "IMU biases estimation was saved to: " << calibrationPath << std::endl;

	return EXIT_SUCCESS;
}

#pragma once

#include "camera_calibration.hpp"

#include <span>

namespace dv::camera::calibrations {

struct IMUCalibration {
	struct Metadata {
		/// Timestamp when the calibration was conducted
		std::string calibrationTime;

		/// Any additional information
		std::string comment;

		explicit Metadata(const std::string &calibrationTime = "", const std::string &comment = "") :
			calibrationTime(calibrationTime),
			comment(comment) {
		}

		explicit Metadata(const pt::ptree &tree) :
			Metadata(tree.get_child("calibrationTime").get_value<std::string>(""),
				tree.get_child("comment").get_value<std::string>("")) {
		}

		[[nodiscard]] pt::ptree toPropertyTree() const {
			pt::ptree tree;
			tree.put("calibrationTime", calibrationTime);
			tree.put("comment", comment);
			return tree;
		}

		bool operator==(const Metadata &rhs) const {
			return calibrationTime == rhs.calibrationTime && comment == rhs.comment;
		}
	};

	/// Sensor name (e.g. "IMU_DVXplorer_DXA02137")
	std::string name;

	/// Maximum (saturation) angular velocity of the gyroscope [rad/s]
	float omegaMax = -1.f;

	/// Maximum (saturation) acceleration of the accelerometer [m/s^2]
	float accMax = -1.f;

	/// Average offset (bias) of the angular velocity [rad/s]
	cv::Point3f omegaOffsetAvg;

	/// Average offset (bias) of the acceleration [m/s^2]
	cv::Point3f accOffsetAvg;

	/// Variance of the offset of the angular velocity [rad/s]
	float omegaOffsetVar = -1.f;

	/// Variance of the offset of the acceleration [m/s^2]
	float accOffsetVar = -1.f;

	/// Noise density of the gyroscope [rad/s^s/sqrt(Hz)]
	float omegaNoiseDensity = -1.f;

	/// Noise density of the accelerometer [m/s^2/sqrt(Hz)]
	float accNoiseDensity = -1.f;

	/// Noise random walk of the gyroscope [rad/s^s/sqrt(Hz)]
	float omegaNoiseRandomWalk = -1.f;

	/// Noise random walk of the accelerometer [m/s^2/sqrt(Hz)]
	float accNoiseRandomWalk = -1.f;

	/// Offset between the camera and IMU timestamps in microseconds (t_correct = t_imu - offset)
	int64_t timeOffsetMicros = -1;

	/// Transformation converting points in IMU frame to C0 frame p_C0= T * p_IMU
	std::vector<float> transformationToC0;

	/// Metadata
	std::optional<Metadata> metadata;

	IMUCalibration() = default;

	IMUCalibration(const std::string &name, const float omegaMax, const float accMax, const cv::Point3f &omegaOffsetAvg,
		const cv::Point3f &accOffsetAvg, const float omegaOffsetVar, const float accOffsetVar,
		const float omegaNoiseDensity, const float accNoiseDensity, const float omegaNoiseRandomWalk,
		const float accNoiseRandomWalk, const int64_t timeOffsetMicros, std::span<const float> transformationToC0View,
		const std::optional<Metadata> &metadata) :
		name(name),
		omegaMax(omegaMax),
		accMax(accMax),
		omegaOffsetAvg(omegaOffsetAvg),
		accOffsetAvg(accOffsetAvg),
		omegaOffsetVar(omegaOffsetVar),
		accOffsetVar(accOffsetVar),
		omegaNoiseDensity(omegaNoiseDensity),
		accNoiseDensity(accNoiseDensity),
		omegaNoiseRandomWalk(omegaNoiseRandomWalk),
		accNoiseRandomWalk(accNoiseRandomWalk),
		timeOffsetMicros(timeOffsetMicros),
		transformationToC0(transformationToC0View.begin(), transformationToC0View.end()),
		metadata(metadata) {
		CameraCalibration::validateTransformation(transformationToC0);
	}

	explicit IMUCalibration(const pt::ptree &tree) :
		IMUCalibration(tree.get_child("name").get_value<std::string>(""),
			tree.get_child("omegaMax").get_value<float>(-1.f), tree.get_child("accMax").get_value<float>(-1.f),
			CameraCalibration::parseTripple<cv::Point3f, float>(tree, "omegaOffsetAvg"),
			CameraCalibration::parseTripple<cv::Point3f, float>(tree, "accOffsetAvg"),
			tree.get_child("omegaOffsetVar").get_value<float>(-1.f),
			tree.get_child("accOffsetVar").get_value<float>(-1.f),
			tree.get_child("omegaNoiseDensity").get_value<float>(-1.f),
			tree.get_child("accNoiseDensity").get_value<float>(-1.f),
			tree.get_child("omegaNoiseRandomWalk").get_value<float>(-1.f),
			tree.get_child("accNoiseRandomWalk").get_value<float>(-1.f),
			tree.get_child("timeOffsetMicros").get_value<int64_t>(-1),
			CameraCalibration::getVectorFromTree<float>("transformationToC0", tree),
			CameraCalibration::getOptionalMetadata<Metadata>(tree, "metadata")) {
		CameraCalibration::validateTransformation(transformationToC0);
	}

	[[nodiscard]] pt::ptree toPropertyTree() const {
		pt::ptree tree;
		tree.put("name", name);
		tree.put("omegaMax", omegaMax);
		tree.put("accMax", accMax);
		CameraCalibration::pushVectorToTree<float>(
			"omegaOffsetAvg", {omegaOffsetAvg.x, omegaOffsetAvg.y, omegaOffsetAvg.z}, tree);
		CameraCalibration::pushVectorToTree<float>(
			"accOffsetAvg", {accOffsetAvg.x, accOffsetAvg.y, accOffsetAvg.z}, tree);
		tree.put("omegaOffsetVar", omegaOffsetVar);
		tree.put("accOffsetVar", accOffsetVar);
		tree.put("omegaNoiseDensity", omegaNoiseDensity);
		tree.put("accNoiseDensity", accNoiseDensity);
		tree.put("omegaNoiseRandomWalk", omegaNoiseRandomWalk);
		tree.put("accNoiseRandomWalk", accNoiseRandomWalk);
		tree.put("timeOffsetMicros", timeOffsetMicros);
		CameraCalibration::pushVectorToTree("transformationToC0", transformationToC0, tree);
		if (metadata.has_value()) {
			tree.put_child("metadata", metadata->toPropertyTree());
		}
		return tree;
	}

	bool operator==(const IMUCalibration &rhs) const {
		return name == rhs.name && omegaMax == rhs.omegaMax && accMax == rhs.accMax
			&& omegaOffsetAvg == rhs.omegaOffsetAvg && accOffsetAvg == rhs.accOffsetAvg
			&& omegaOffsetVar == rhs.omegaOffsetVar && accOffsetVar == rhs.accOffsetVar
			&& omegaNoiseDensity == rhs.omegaNoiseDensity && accNoiseDensity == rhs.accNoiseDensity
			&& omegaNoiseRandomWalk == rhs.omegaNoiseRandomWalk && accNoiseRandomWalk == rhs.accNoiseRandomWalk
			&& timeOffsetMicros == rhs.timeOffsetMicros && transformationToC0 == rhs.transformationToC0
			&& metadata == rhs.metadata;
	}

	friend std::ostream &operator<<(std::ostream &os, const IMUCalibration &calibration) {
		os << "name: " << calibration.name << std::endl;
		os << " omegaMax: " << calibration.omegaMax << std::endl;
		os << " accMax: " << calibration.accMax << std::endl;
		os << " omegaOffsetAvg: " << calibration.omegaOffsetAvg << std::endl;
		os << " accOffsetAvg: " << calibration.accOffsetAvg << std::endl;
		os << " omegaOffsetVar: " << calibration.omegaOffsetVar << std::endl;
		os << " accOffsetVar: " << calibration.accOffsetVar << std::endl;
		os << " omegaNoiseDensity: " << calibration.omegaNoiseDensity << std::endl;
		os << " accNoiseDensity: " << calibration.accNoiseDensity << std::endl;
		os << " omegaNoiseRandomWalk: " << calibration.omegaNoiseRandomWalk << std::endl;
		os << " accNoiseRandomWalk: " << calibration.accNoiseRandomWalk << std::endl;
		os << " timeOffsetMicros: " << calibration.timeOffsetMicros << std::endl;
		os << " transformationToC0: " << fmt::format("[{}]", fmt::join(calibration.transformationToC0, ", "))
		   << std::endl;
		return os;
	}
};

} // namespace dv::camera::calibrations

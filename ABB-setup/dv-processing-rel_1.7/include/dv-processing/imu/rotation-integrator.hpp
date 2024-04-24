#pragma once

#include "../core/concepts.hpp"
#include "../data/imu_base.hpp"
#include "../kinematics/transformation.hpp"

#include <Eigen/Geometry>

#include <numbers>

namespace dv::imu {

class RotationIntegrator {
private:
	/**
	 * matrix storing target position wrt to the sensor (imu)
	 */
	Eigen::Matrix4f mT_S0_target;

	/**
	 * offset [us] between sensor and target: t_target = t_sensor - offset
	 */
	int64_t mSensorToTargetTimeOffset;

	/**
	 * measurement offset [radians] along each x, y, z axis of the sensor
	 */
	Eigen::Vector3f mGyroscopeOffset;

	/**
	 * matrix storing current sensor orientation wrt initial sensor orientation
	 */
	Eigen::Matrix3f mR_S0_S = Eigen::Matrix3f::Identity(3, 3);

	/**
	 * timestamp of current sensor position wrt initial time.
	 */
	int64_t mTimestamp = -1;

	/**
	 * Transform gyroscope measurement into rotation matrix representation
	 * @param imu single imu measurement
	 * @return [3x3] rotation matrix corresponding to rotation measured from gyroscope
	 */
	[[nodiscard]] Eigen::Matrix3f rotationMatrixFromImu(const dv::IMU &imu, const float dt) {
		// deg2rad conversion
		Eigen::Vector3f imuRadiant = (imu.getAngularVelocities() - mGyroscopeOffset) * dt;
		// gyroscope data as euler angles
		Eigen::AngleAxisf rollAngle(imuRadiant.x(), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf pitchAngle(imuRadiant.y(), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf yawAngle(imuRadiant.z(), Eigen::Vector3f::UnitZ());
		// quaternion from euler angles
		Eigen::Quaternionf quaternion = yawAngle * pitchAngle * rollAngle;
		return quaternion.matrix();
	}

public:
	/**
	 *
	 * @param T_S_target initial target position wrt to sensor
	 * @param sensorToTargetTimeOffset temporal offset between sensor (imu) and target. t_target = t_sensor - offset
	 * @param gyroscopeOffset constant measurement offset in gyroscope samples [radians].
	 */
	explicit RotationIntegrator(const dv::kinematics::Transformationf &T_S_target = dv::kinematics::Transformationf(),
		int64_t sensorToTargetTimeOffset = 0, const Eigen::Vector3f &gyroscopeOffset = {0.f, 0.f, 0.f}) :
		mT_S0_target(T_S_target.getTransform()),
		mSensorToTargetTimeOffset(sensorToTargetTimeOffset),
		mGyroscopeOffset(gyroscopeOffset) {
	}

	/**
	 * Getter outputting current target transformation relative to (target) initial one
	 * @return [3x3] rotation matrix
	 */
	[[nodiscard]] Eigen::Matrix3f getRotation() const {
		if (mTimestamp == -1) {
			throw std::invalid_argument("Trying to get camera orientation even if no imu data was integrated");
		}
		return mR_S0_S * mT_S0_target.block<3, 3>(0, 0).transpose();
	}

	/**
	 * Setter to update target position wrt to the sensor
	 * @param T_S_target new target transformation wrt sensor
	 */
	void setT_S_target(const dv::kinematics::Transformationf &T_S_target) {
		mT_S0_target = T_S_target.getTransform();
	}

	/**
	 * Getter outputting timestamp of current target transformation
	 * @return timestamp
	 */
	[[nodiscard]] int64_t getTimestamp() const {
		return mTimestamp;
	}

	/**
	 * Getter returning [4x4] transformation corresponding to current target position wrt (target) initial one
	 * @return 4x4 transformation corresponding to current integrated rotation
	 */
	[[nodiscard]] dv::kinematics::Transformation<float> getTransformation() const {
		if (mTimestamp == -1) {
			throw std::invalid_argument("Trying to get camera position even if no imu data was integrated");
		}
		Eigen::Matrix<float, 4, 4> T_target_S;
		Eigen::Matrix<float, 4, 4> T_S0_S = Eigen::Matrix<float, 4, 4>::Identity(4, 4);

		T_S0_S.block<3, 3>(0, 0) = mR_S0_S;

		T_target_S = T_S0_S * mT_S0_target.transpose();

		dv::kinematics::Transformation<float> dvTransformation = {mTimestamp, T_target_S};
		return dvTransformation;
	}

	/**
	 * Update sensor position with new measurement
	 * @param imu single imu measurement
	 */
	void accept(const dv::IMU &imu) {
		int64_t timestamp = imu.timestamp - mSensorToTargetTimeOffset;

		if (mTimestamp >= 0) {
			mR_S0_S *= rotationMatrixFromImu(imu, static_cast<float>(timestamp - mTimestamp) * 1e-6f);
		}

		mTimestamp = timestamp;
	}
};

static_assert(dv::concepts::Accepts<RotationIntegrator, dv::IMU>);

} // namespace dv::imu

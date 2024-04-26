//
// Created by giovanni on 11/10/21.
//

#include "dv-processing/imu/rotation-integrator.hpp"

#include "boost/ut.hpp"

#include <iostream>

static dv::IMU generateImuSample(int64_t timestamp, float x, float y, float z) {
	//	 generate samples in degrees (expected angular speed to orientation integration already happened)
	dv::IMU imu;
	imu.gyroscopeX = x;
	imu.gyroscopeY = y;
	imu.gyroscopeZ = z;
	imu.timestamp  = timestamp;
	return imu;
}

int main() {
	using namespace boost::ut;

	"init rotation expected failure"_test = []() {
		// INIT ROTATION TEST: EXPECTED FAILURE TRYING TO GET CAMERA ORIENTATION BEFORE FEEDING ANY IMU DATA
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf();
		Eigen::Matrix3f randomInit                       = Eigen::Matrix3f::Random(3, 3);
		dv::imu::RotationIntegrator accumulateRotation{T_S_target, 0};
		expect(throws([&] {
			// Throws an exception here
			auto _ = accumulateRotation.getRotation();
		}));
	};

	"init rotation correct"_test = []() {
		// INIT ROTATION TEST
		dv::IMU imu;
		imu                                              = generateImuSample(1'000, 0.0f, 0.0f, 0.0f);
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf();
		Eigen::Matrix3f randomInit                       = Eigen::Matrix3f::Random(3, 3);
		dv::imu::RotationIntegrator accumulateRotation{T_S_target, 0};
		accumulateRotation.accept(imu);

		expect(accumulateRotation.getRotation().isApprox(Eigen::Matrix3f::Identity(3, 3)));
	};

	"x-axis rotation "_test = []() {
		// ROTATE BY 90 AROUND X AXIS FROM STARTING POSITION
		/* => y-axis points at original z axis direction and z axis point in the opposite direction of 1 axis
		 * => expected final matrix:
		 * 								{	{1, 0, 0},
		 * 									{0, 0, 1},
		 * 									{0, -1, 0},	}
		 *
		 *
		 */
		dv::imu::RotationIntegrator accumulateRotation;
		dv::IMU zero = generateImuSample(1'000'000, 0.f, 0.f, 0.f);
		accumulateRotation.accept(zero);

		dv::IMU imu_pos_x = generateImuSample(1'100'000, 90.f / 0.1f, 0.f, 0.f);
		Eigen::Matrix3f groundTruthMatrix;
		groundTruthMatrix << 1, 0, 0, 0, 0, -1, 0, 1, 0;
		accumulateRotation.accept(imu_pos_x);

		auto estimation = accumulateRotation.getRotation();
		expect(estimation.isApprox(groundTruthMatrix));
	};

	"forward-back rotation "_test = []() {
		// APPLY BACK AND FORWARD ROTATION AND CHECK FINAL POSITION CORRESPONDS TO INITIAL ONE
		dv::IMU initOrientation = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imuPosX         = generateImuSample(1'100'000, 90.f * 0.1f, 0.0f, 0.0f);
		dv::IMU imuNegX         = generateImuSample(1'200'000, -90.f * 0.1f, 0.0f, 0.0f);

		dv::imu::RotationIntegrator accumulateRotation;
		accumulateRotation.accept(initOrientation);
		Eigen::Matrix3f init_position = accumulateRotation.getRotation();
		accumulateRotation.accept(imuPosX);
		accumulateRotation.accept(imuNegX);
		Eigen::Matrix3f end_position = accumulateRotation.getRotation();

		expect(end_position.isApprox(init_position));
	};

	"two-half-rotation-one-doubled-along-axis "_test = []() {
		// APPLY TWO HALF ROTATION AND CHECK EQUIVALENT TO SAME ROTATION WITH DOUBLE VAL

		dv::IMU initOrientation = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imu             = generateImuSample(1'100'000, 90.f * 0.1f, 0.0f, 0.0f);
		dv::IMU imuDoubled      = generateImuSample(1'100'000, 180.f * 0.1f, 0.0f, 0.0f);
		dv::imu::RotationIntegrator firstRotation;
		dv::imu::RotationIntegrator secondRotation;
		firstRotation.accept(initOrientation);
		secondRotation.accept(initOrientation);

		firstRotation.accept(imu);
		imu.timestamp = 1'200'000;
		firstRotation.accept(imu);
		secondRotation.accept(imuDoubled);

		expect(firstRotation.getRotation().isApprox(secondRotation.getRotation()));
	};

	"no-rotation"_test = []() {
		// APPLY NO ROTATION TO CHECK PROPER 0 DIVISION HANDLING
		Eigen::Matrix3f initPosition;
		initPosition = Eigen::Matrix3f::Identity(3, 3);
		dv::IMU imu  = generateImuSample(1'000'000, 0.0f, 0.0f, 0.0f);
		dv::imu::RotationIntegrator accumulateRotation;
		accumulateRotation.accept(imu);

		expect(accumulateRotation.getRotation().isApprox((initPosition)));
	};

	"target_frame_transform"_test = []() {
		// APPLY RANDOM ROTATION AND RETURN 4X4 TRANSFORMATION CORRESPONDING TO IT
		dv::IMU initOrientation = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imu             = generateImuSample(1'100'000, 90.f, 0.0f, 0.0f);
		dv::imu::RotationIntegrator accumulateRotation;
		accumulateRotation.accept(initOrientation);
		accumulateRotation.accept(imu);
		dv::kinematics::Transformation<float> transformation = accumulateRotation.getTransformation();

		expect(transformation.getRotationMatrix().isApprox(accumulateRotation.getRotation()));
		expect(transformation.getTranslation().isApprox(Eigen::Vector3f::Zero(3, 1)));
		expect(transformation.getTimestamp() == accumulateRotation.getTimestamp());
	};

	"imu-target-orientation"_test = []() {
		// TEST CONSISTENCY IF T_S_target DIFFERENT FROM IDENTITY
		// transformation from imu to target as 90 deg rotation around z axis
		Eigen::Matrix4f T_S_target_eigen;
		T_S_target_eigen << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf(0, T_S_target_eigen);

		// imu gets negative rotation of -90 => T_W_target is identity
		dv::IMU initOrientation = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imu             = generateImuSample(1'100'000, 0.0f, 0.0f, 90.f / 0.1f);
		dv::imu::RotationIntegrator accumulateRotation(T_S_target, 0);
		accumulateRotation.accept(initOrientation);
		accumulateRotation.accept(imu);
		dv::kinematics::Transformation<float> transformation = accumulateRotation.getTransformation();

		expect(transformation.getTransform().isApprox(Eigen::Matrix4f::Identity(4, 4)));
	};
	"imu-to-target-transformation"_test = []() {
		// TEST 2 CONSECUTIVE ROTATION OF +60DEG AND +30DEG FROM IMU RESULTS IN IDENTITY TRANSFORMATION AT THE END
		// OF THE TWO ROTATIONS IF TARGET FRAME HAS -90DEG ORIENTATION WRT SENSOR

		// -90deg orientation of target wrt sensor frame
		Eigen::Matrix4f T_S_target_eigen;
		T_S_target_eigen << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf(0, T_S_target_eigen);

		Eigen::Matrix4f T_W_target_intermediate;
		// intermediate transformation is +30deg wrt initial target position -(-90 - (-60))
		T_W_target_intermediate << sqrtf(3.0f) / 2.f, -0.5f, 0, 0, 0.5f, sqrtf(3.0f) / 2.f, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			1;
		dv::IMU initOrientation = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imu1            = generateImuSample(1'100'000, 0.0f, 0.0f, -60.f / 0.1f);
		dv::IMU imu2            = generateImuSample(1'200'000, 0.0f, 0.0f, -30.f / 0.1f);
		dv::imu::RotationIntegrator accumulateRotation(T_S_target, 0);
		accumulateRotation.accept(initOrientation);
		accumulateRotation.accept(imu1);
		expect(accumulateRotation.getTransformation().getTransform().isApprox(T_W_target_intermediate));
		accumulateRotation.accept(imu2);
		dv::kinematics::Transformation<float> transformation = accumulateRotation.getTransformation();

		expect(transformation.getTransform().isApprox(Eigen::Matrix4f::Identity(4, 4)));
	};
	"T_s_target-setter"_test = []() {
		// TEST CORRECTNESS OF T_s_target

		dv::IMU imu = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::imu::RotationIntegrator accumulateRotation;
		accumulateRotation.accept(imu);
		expect(accumulateRotation.getTransformation().getTransform().isApprox(Eigen::Matrix4f::Identity(4, 4)));
		// new -90deg orientation of target wrt sensor frame
		Eigen::Matrix4f T_S_target_eigen;
		T_S_target_eigen << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf(0, T_S_target_eigen);
		accumulateRotation.setT_S_target(T_S_target);
		expect(accumulateRotation.getTransformation().getTransform().isApprox(T_S_target_eigen.transpose()));
	};

	"imu-offset"_test = []() {
		// TEST IMU OFFSET
		dv::IMU init = generateImuSample(1'000'000, 0.f, 0.0f, 0.0f);
		dv::IMU imu  = generateImuSample(1'100'000, 2.3f, 1.0f, 5.0f);

		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf();

		Eigen::Matrix3f identity = Eigen::Matrix3f::Identity(3, 3);

		Eigen::Vector3f gyroscopeOffsetRad(0.0401426f, 0.0174533f, 0.0872665f);
		dv::imu::RotationIntegrator accumulateRotation{T_S_target, 0, gyroscopeOffsetRad};

		accumulateRotation.accept(imu);

		expect(accumulateRotation.getRotation().isApprox(identity));
	};

	"imu-to-target-time-offset"_test = []() {
		// TEST TIME OFFSET BETWEEN IMU AND TARGET
		const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf();

		Eigen::Matrix3f identity = Eigen::Matrix3f::Identity(3, 3);

		const int64_t imuToTargetTimeOffset = 512;

		dv::imu::RotationIntegrator accumulateRotation{T_S_target, imuToTargetTimeOffset};
		dv::IMU imu = generateImuSample(imuToTargetTimeOffset, 2.3f, 1.0f, 5.0f);
		accumulateRotation.accept(imu);

		// ts_target = ts_imu - offset
		expect(accumulateRotation.getTimestamp() == 0);
	};

	return EXIT_SUCCESS;
}

//
// Created by giovanni on 26.04.22.
//

#pragma once
#include "../camera/camera_geometry.hpp"
#include "../core/core.hpp"
#include "../imu/rotation-integrator.hpp"
#include "../kinematics/motion_compensator.hpp"
#include "../optimization/optimization_functor.hpp"

namespace dv::optimization {

/**
 * Given a chunk of events, the idea of contrast maximization is to warp events in space and time given a predefined
 * motion model. Contrast maximization aims at finding the optimal parameters of the given motion model.
 * The idea is that if the motion is perfectly estimated, all events corresponding to the same point in the scene, will
 * be warped to the same image plane location, at a given point in time. If this happens, the reconstructed event image
 * will be sharp, having high contrast. This high contrast is measured as variance in the image. For this reason,
 * contrast maximization searches for the best motion parameters which maximize the contrast of the event image
 * reconstructed after warping events in space to a spacific point in time. In order to warp event in space and time we
 * use the "dv::kinematics::MotionCompensator" class.
 * This contrast maximization class assumes pure camera rotational motion model. Given a set of imu samples and events
 * in a time range, gyroscope measurement offset if optimized. The gyroscope offset is optimized instead of each single
 * gyroscope measurement in order to limit the search space of the non linear optimization. In addition, given the high
 * sample rate of imu, it would be hard to achieve real time computing optimizing each single gyroscope value. For this
 * reason, the gyroscope offset (x, y, z) is optimized and assumed to be constant among all the gyroscope samples.
 */
class RotationLossFunctor : public OptimizationFunctor<float> {
private:
	/**
	 * Camera geometry data. This information is used to create motionCompensator and compensate events.
	 */
	dv::camera::CameraGeometry::SharedPtr mCamera;

	/**
	 * Raw events compensated using imu data.
	 */
	const dv::EventStore mEvents;

	/**
	 * Event contribution for total pixel intensity. This parameter is very important since it strongly influence
	 * contrast value. It needs to be tuned based on scene and length of event chunk.
	 */
	float mContribution;

	/**
	 * Imu data used to compensate mEvents.
	 */
	const dv::cvector<dv::IMU> mImuSamples;

	/**
	 * Target (i.e. camera) to imu transformation. Used to construct rotationIntegrator that keeps track of camera
	 * position.
	 */
	const dv::kinematics::Transformationf mT_S_target;

	/**
	 * Time offset between imu and target. Check rotationIntegrator class for more information.
	 */
	int64_t mImuToTargetTimeOffsetUs;

public:
	/**
	 * This contrast maximization class assumes pure camera rotational motion model. Given a set of imu samples and
	 * events in a time range, gyroscope measurement offset if optimized. The gyroscope offset is optimized instead of
	 * each single gyroscope measurement in order to limit the search space of the non linear optimization.
	 * @param camera Camera geometry used to create motion compensator
	 * @param events Events used to compute motion compensated image
	 * @param contribution Contribution value of each event to the total pixel intensity
	 * @param imuSamples Chunk of imu samples used to compensate events. These values (gyrosvcope part) are updated with
	 * the gyroscope measurement offset, which is the optimized variable.
	 * @param T_S_target Transformation from sensor (imu) to target (camera). Used to convert imu motion into camera
	 * motion.
	 * @param imuToCamTimeOffsetUs Time synchronization offset between imu and camera
	 * @param inputDim Number of parameters to optimize
	 * @param numMeasurements Number of function evaluation performed to compute the gradient
	 */
	RotationLossFunctor(dv::camera::CameraGeometry::SharedPtr &camera, const dv::EventStore &events, float contribution,
		const dv::cvector<dv::IMU> &imuSamples, const dv::kinematics::Transformationf &T_S_target,
		int64_t imuToCamTimeOffsetUs, int inputDim, int numMeasurements) :
		OptimizationFunctor<float>(inputDim, numMeasurements),
		mCamera(camera),
		mEvents(events),
		mContribution(contribution),
		mImuSamples(imuSamples),
		mT_S_target(T_S_target),
		mImuToTargetTimeOffsetUs(imuToCamTimeOffsetUs) {
		if (numMeasurements < inputDim) {
			throw dv::exceptions::InvalidArgument<int>(fmt::format(
				"Optimization is ill-posed: number of measurements {} should be >= number of variables to optimize {}.",
				numMeasurements, inputDim));
		}
		if (events.getLowestTime() < imuSamples.front().timestamp) {
			throw dv::exceptions::InputError("Events timestamp smaller than smallest imu timestamp: motion compensator "
											 "will not be able to perform compensation");
		}
		if (events.getHighestTime() > imuSamples.back().timestamp) {
			throw dv::exceptions::InputError("Events timestamp bigger than biggest imu timestamp: motion compensator "
											 "will not be able to perform compensation");
		}
	}

	/**
	 * Implementation of the objective function: optimize gyroscope offset.
	 * Current cost is stored in stdInverse. Notice that since we want to maximize the contrast
	 * but optimizer minimize cost function we use as cost 1/contrast
	 */
	int operator()(const Eigen::VectorXf &gyroscopeOffsetImu, Eigen::VectorXf &stdInverse) const {
		// create compensator and feed transformations and events
		std::unique_ptr<dv::kinematics::MotionCompensator<>> mc = std::make_unique<dv::kinematics::MotionCompensator<>>(
			mCamera, std::make_unique<dv::EdgeMapAccumulator>(mCamera->getResolution(), mContribution, true));
		mc->accept(mEvents);

		// create rotation integrator
		Eigen::Vector3f gyroscopeOffsetImuFloat;
		gyroscopeOffsetImuFloat << static_cast<float>(gyroscopeOffsetImu.x()),
			static_cast<float>(gyroscopeOffsetImu.y()), static_cast<float>(gyroscopeOffsetImu.z());
		dv::imu::RotationIntegrator rotationIntegrator(mT_S_target, mImuToTargetTimeOffsetUs, gyroscopeOffsetImuFloat);

		// integrate imu & feed transformations to motion compensator
		for (const auto &imu : mImuSamples) {
			rotationIntegrator.accept(imu);
			auto transform = rotationIntegrator.getTransformation();
			mc->accept(transform);
		}

		// warp events and generate event image after warping
		dv::Frame motionCompensatedToStart = mc->generateFrame(mEvents.getHighestTime());
		if (not mc->getInfo().imageCompensated) {
			throw dv::exceptions::RuntimeError(
				"Event chunk is not compensated, check your motion compensator parameters");
		}
		// get event image
		cv::Mat eventImage = motionCompensatedToStart.image;
		// compute standard deviation
		cv::Mat mean, stddev;
		cv::meanStdDev(eventImage, mean, stddev);

		stdInverse(0) = 1 / stddev.at<double>(0);
		stdInverse(1) = 0;
		stdInverse(2) = 0;

		return 0;
	}
};

} // namespace dv::optimization

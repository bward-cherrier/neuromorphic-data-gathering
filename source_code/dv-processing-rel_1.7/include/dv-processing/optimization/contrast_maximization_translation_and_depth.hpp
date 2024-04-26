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
 * reconstructed after warping events in space to a specific point in time. In order to warp event in space and time we
 * use the "dv::kinematics::MotionCompensator" class.
 * This contrast maximization class assumes pure camera translation motion model.
 * Given a set of events in a time range (init_time, end_time), assuming a constant translational speed between
 * init_time and end_time, translation (x, y, z) and scene depth are optimized to maximize contrast of event image.
 * Since the speed is assumed to be constant between init_time end end_time, the camera position at time t_k is computed
 * as : t_k = speed*dt, where dt = t_k - init_time. The scene depth is included in the optimization since it is strongly
 * correlated to the camera translation. Scene depth is assumed to be constant between init_time and end_time.
 */
class TranslationLossFunctor : public OptimizationFunctor<float> {
private:
	/**
	 * Camera geometry data. This information is used to create motionCompensator and compensate events.
	 */
	dv::camera::CameraGeometry::SharedPtr mCamera;

	/**
	 * Raw events compensated using translation along x, y, z and current scene depth.
	 */
	const dv::EventStore mEvents;

	/**
	 * Event contribution for total pixel intensity. This parameter is very important since it strongly influence
	 * contrast value. It needs to be tuned based on scene and length of event chunk.
	 */
	const float mContribution;

public:
	/**
	 * This contrast maximization class assumes pure camera translation motion model.
	 * Given a set of events in a time range (init_time, end_time), assuming a constant translational speed between
	 * init_time and end_time, translation (x, y, z) and scene depth are optimized to maximize contrast of event image.
	 * @param camera Camera geometry used to create motion compensator
	 * @param events Events used to compute motion compensated image
	 * @param contribution Contribution value of each event to the total pixel intensity
	 * @param inputDim Number of parameters to optimize
	 * @param numMeasurements Number of function evaluation performed to compute the gradient
	 */
	TranslationLossFunctor(dv::camera::CameraGeometry::SharedPtr &camera, const dv::EventStore &events,
		float contribution, int inputDim, int numMeasurements) :
		OptimizationFunctor<float>(inputDim, numMeasurements),
		mCamera(camera),
		mEvents(events),
		mContribution(contribution) {
		if (numMeasurements < inputDim) {
			throw dv::exceptions::InvalidArgument<int>(fmt::format(
				"Optimization is ill-posed: number of measurements {} should be >= number of variables to optimize {}.",
				numMeasurements, inputDim));
		}
	}

	/**
	 * Implementation of the objective function: optimize camera translation (x, y, z) and scene depth.
	 * Current cost is stored in stdInverse. Notice that since we want to maximize the contrast
	 * but optimizer minimize cost function we use as cost 1/contrast
	 */
	int operator()(const Eigen::VectorXf &translationAndDepth, Eigen::VectorXf &stdInverse) const {
		Eigen::Vector3f translation = translationAndDepth.block<3, 1>(0, 0);
		float depth                 = std::max(translationAndDepth.w(), 0.f);

		// create compensator
		std::unique_ptr<dv::kinematics::MotionCompensator<>> mc = std::make_unique<dv::kinematics::MotionCompensator<>>(
			mCamera, std::make_unique<dv::EdgeMapAccumulator>(mCamera->getResolution(), mContribution, true));
		// feed events to motion compensator
		mc->accept(mEvents);

		// feed transformation to motion compensator
		Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix4f::Identity();
		transform.block<3, 1>(0, 3)          = translation;
		const dv::kinematics::Transformation<float> initTransform
			= {mEvents.getLowestTime(), Eigen::Matrix4f::Identity()};
		dv::kinematics::Transformation<float> endTransform = {mEvents.getHighestTime(), transform};
		mc->accept(initTransform);
		mc->accept(endTransform);

		mc->setConstantDepth(depth);

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
		stdInverse(3) = 0;

		return 0;
	}
};
} // namespace dv::optimization

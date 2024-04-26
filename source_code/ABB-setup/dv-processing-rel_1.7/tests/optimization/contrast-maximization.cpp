//
// Created by giovanni on 27.04.22.
//
#include "dv-processing/optimization/contrast_maximization_rotation.hpp"
#include "dv-processing/optimization/contrast_maximization_translation_and_depth.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;
	using namespace dv::optimization;

	"translation_loss_functor"_test = [&] {
		int inputDim        = 3;
		int numMeasurements = 4;
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::EventStore events;
		events.emplace_back(10000, 20, 20, true);
		expect(nothrow([&] {
			TranslationLossFunctor(camera, events, 0.01f, inputDim, numMeasurements);
		}));
	};

	"translation_loss_functor_wrong_input_output_dimension"_test = [&] {
		int inputDim        = 3;
		int numMeasurements = 2;
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::EventStore events;
		events.emplace_back(10000, 20, 20, true);
		expect(throws([&] {
			TranslationLossFunctor(camera, events, 0.01f, inputDim, numMeasurements);
		}));
	};

	"rotation_loss_functor"_test = [&] {
		int inputDim        = 3;
		int numMeasurements = 4;
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::EventStore events;
		events.emplace_back(10000, 20, 20, true);
		auto imu = dv::IMU(10000, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
		dv::cvector<dv::IMU> imuSamples;
		imuSamples.push_back(imu);
		dv::kinematics::Transformationf transform(10000, Eigen::Matrix4f::Identity());

		expect(nothrow([&] {
			RotationLossFunctor(camera, events, 0.01f, imuSamples, transform, 0, inputDim, numMeasurements);
		}));
	};

	"rotation_loss_functor_wrong_input_output_dimension"_test = [&] {
		int inputDim        = 3;
		int numMeasurements = 2;
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::EventStore events;
		events.emplace_back(10000, 20, 20, true);
		auto imu = dv::IMU(10000, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
		dv::cvector<dv::IMU> imuSamples;
		imuSamples.push_back(imu);
		dv::kinematics::Transformationf transform(10000, Eigen::Matrix4f::Identity());

		expect(throws([&] {
			RotationLossFunctor(camera, events, 0.01f, imuSamples, transform, 0, inputDim, numMeasurements);
		}));
	};

	"rotation_loss_functor_out_of_bounds_events"_test = [&] {
		int inputDim        = 3;
		int numMeasurements = 3;
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::EventStore eventsBeforeImu;
		dv::EventStore eventsInsideImuRange;
		dv::EventStore eventsAfterImu;
		eventsBeforeImu.emplace_back(10000, 20, 20, true);
		eventsInsideImuRange.emplace_back(15000, 20, 20, true);
		eventsAfterImu.emplace_back(20000, 20, 20, true);
		auto imu1 = dv::IMU(10001, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
		auto imu2 = dv::IMU(19999, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
		dv::cvector<dv::IMU> imuSamples;
		imuSamples.push_back(imu1);
		imuSamples.push_back(imu2);

		dv::kinematics::Transformationf transform(10000, Eigen::Matrix4f::Identity());

		expect(throws([&] {
			RotationLossFunctor(camera, eventsBeforeImu, 0.01f, imuSamples, transform, 0, inputDim, numMeasurements);
		}));

		expect(nothrow([&] {
			RotationLossFunctor(
				camera, eventsInsideImuRange, 0.01f, imuSamples, transform, 0, inputDim, numMeasurements);
		}));

		expect(throws([&] {
			RotationLossFunctor(camera, eventsAfterImu, 0.01f, imuSamples, transform, 0, inputDim, numMeasurements);
		}));
	};
}

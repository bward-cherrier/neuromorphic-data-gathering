//
// Created by rokas on 19.08.21.
// Copyright (c) 2021 iniVation AG. All rights reserved.
//

#include "../../include/dv-processing/camera/calibrations/camera_calibration.hpp"

#include "boost/ut.hpp"

#include <filesystem>

int main(int ac, char **av) {
	using namespace boost::ut;
	const float fx = 222.395355f;
	const float fy = 222.470474f;
	const float cx = 175.044815f;
	const float cy = 139.301224f;
	const cv::Size resolution(346, 260);

	"radTan_distortion"_test = [&] {
		const Eigen::Vector3f point(1.f, 2.f, 5.f);

		const std::vector<float> distortion{0.002f, 0.00999999978f, 0.0199999996f, 0.0299999993f, 0.03999999f};

		auto camera = std::make_unique<dv::camera::CameraGeometry>(
			distortion, fx, fy, cx, cy, resolution, dv::camera::DistortionModel::RadTan);

		// Consistency test
		const Eigen::Vector2f pointProjected = camera->project<Eigen::Vector2f>(point);

		expect(eq(pointProjected.x(), 219.524_f));
		expect(eq(pointProjected.y(), 228.289_f));

		const Eigen::Vector3f distortedPoint = camera->distort<Eigen::Vector3f>(point);

		expect(eq(distortedPoint.x(), 1.0_f));
		expect(eq(distortedPoint.y(), 2.0_f));
		expect(eq(distortedPoint.z(), 5.0_f));

		const Eigen::Vector2f pointProjectedDistorted = camera->project<Eigen::Vector2f>(distortedPoint);

		expect(eq(pointProjectedDistorted.x(), 222._f));
		expect(eq(pointProjectedDistorted.y(), 232._f));

		const Eigen::Vector3f pointEstimated
			= camera->backProjectUndistort<Eigen::Vector3f>(pointProjectedDistorted) * point.z();

		expect(le(std::abs(pointEstimated.x() - point.x()), 0.02));
		expect(le(std::abs(pointEstimated.y() - point.y()), 0.02));
		expect(le(std::abs(pointEstimated.z() - point.z()), 0.02));
	};

	"equidistant_distortion"_test = [&] {
		const Eigen::Vector3f point(1.f, 2.f, 5.f);
		const std::vector<float> distortion{-0.0234784577f, 0.000686703075f, -0.00812580902f, 0.00519397343f};

		auto camera = std::make_unique<dv::camera::CameraGeometry>(
			distortion, fx, fy, cx, cy, resolution, dv::camera::DistortionModel::Equidistant);

		// Consistency test
		const Eigen::Vector2f pointProjected = camera->project<Eigen::Vector2f>(point);

		expect(eq(pointProjected.x(), 219.524_f));
		expect(eq(pointProjected.y(), 228.289_f));

		const Eigen::Vector3f distortedPoint = camera->distort<Eigen::Vector3f>(point);

		expect(eq(distortedPoint.x(), 0.9_f));
		expect(eq(distortedPoint.y(), 1.8_f));
		expect(eq(distortedPoint.z(), 5.0_f));

		const Eigen::Vector2f pointProjectedDistorted = camera->project<Eigen::Vector2f>(distortedPoint);

		expect(eq(pointProjectedDistorted.x(), 216._f));
		expect(eq(pointProjectedDistorted.y(), 222._f));

		const Eigen::Vector3f pointEstimated
			= camera->backProjectUndistort<Eigen::Vector3f>(pointProjectedDistorted) * point.z();

		expect(le(std::abs(pointEstimated.x() - point.x()), 0.02));
		expect(le(std::abs(pointEstimated.y() - point.y()), 0.02));
		expect(le(std::abs(pointEstimated.z() - point.z()), 0.02));
	};
}

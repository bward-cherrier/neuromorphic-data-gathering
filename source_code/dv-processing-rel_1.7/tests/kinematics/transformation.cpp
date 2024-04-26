//
// Created by rokas on 28.07.21.
//

#include "../../include/dv-processing/kinematics/transformation.hpp"

#include "boost/ut.hpp"

#include <Eigen/Geometry>

int main() {
	using namespace boost::ut;
	using namespace dv::kinematics;

	// Test build-up
	Eigen::Quaternionf quaternion(1.f, 0.f, 0.f, 0.f);
	Eigen::Vector3f point(1.f, 2.f, 3.f);
	Transformationf transformation(0, point, quaternion);

	"sanity_checks"_test = [&] {
		// Sanity check whether values are correct
		const auto pointCV = transformation.getTranslation<cv::Point3f>();
		expect(eq(point.x(), pointCV.x));
		expect(eq(point.y(), pointCV.y));
		expect(eq(point.z(), pointCV.z));

		// Sanity check 2.
		const Eigen::Vector3f pointEigen = transformation.getTranslation();
		expect(eq(point.x(), pointEigen.x()));
		expect(eq(point.y(), pointEigen.y()));
		expect(eq(point.z(), pointEigen.z()));
	};

	"point_transformation"_test = [&] {
		// Transformation against the same point without rotation should yield the coordinate addition
		const Eigen::Vector3f transformedPoint = transformation.transformPoint(point);
		expect(eq(point.x() * 2.f, transformedPoint.x()));
		expect(eq(point.y() * 2.f, transformedPoint.y()));
		expect(eq(point.z() * 2.f, transformedPoint.z()));

		// No rotation should return the same point
		const Eigen::Vector3f rotatedPoint = transformation.rotatePoint(point);
		expect(eq(point.x(), rotatedPoint.x()));
		expect(eq(point.y(), rotatedPoint.y()));
		expect(eq(point.z(), rotatedPoint.z()));
	};

	"rotation"_test = [point] {
		// Initialize a matrix with a permutation rotation with no translation
		Eigen::Matrix4f transform;
		transform.setZero();
		transform(1, 0) = 1.f;
		transform(0, 2) = 1.f;
		transform(2, 1) = 1.f;
		transform(3, 3) = 1.f;
		Transformationf permutation(0, transform);
		const Eigen::Vector3f permuted = permutation.transformPoint(point);
		expect(eq(point.x(), permuted.y()));
		expect(eq(point.y(), permuted.z()));
		expect(eq(point.z(), permuted.x()));
	};

	"deltaTo"_test = [transformation] {
		const Eigen::Vector3f pC1(1.f, 1.f, 1.f);

		const Transformationf &T_WC0 = transformation;
		const Transformationf T_C0W  = T_WC0.inverse();

		const Transformationf T_WC1(
			0, Eigen::Vector3f(4.f, 5.f, 6.f), Eigen::Quaternionf(0.9689124f, 0.1428387f, 0.1428387f, 0.1428387f));
		const Transformationf T_C1_C0Pred = T_WC1.delta(T_WC0);
		const Transformationf T_C1_C0(0, T_C0W.getTransform() * T_WC1.getTransform());

		const Eigen::Vector3f pC0Predicted = T_C1_C0Pred.transformPoint(pC1);
		const Eigen::Vector3f pC0          = T_C1_C0.transformPoint(pC1);

		const float dist = (pC0 - pC0Predicted).norm();
		expect(lt(dist, 0.001f));
	};

	"deltaFrom"_test = [transformation] {
		const Eigen::Vector3f pC0(1.f, 1.f, 1.f);

		const Transformationf &T_WC0 = transformation;

		const Transformationf T_WC1(
			0, Eigen::Vector3f(4.f, 5.f, 6.f), Eigen::Quaternionf(0.9689124f, 0.1428387f, 0.1428387f, 0.1428387f));
		const Transformationf T_C1W = T_WC1.inverse();

		const Transformationf T_C0_C1Pred = T_WC0.delta(T_WC1);
		const Transformationf T_C0_C1(0, T_C1W.getTransform() * T_WC0.getTransform());

		const Eigen::Vector3f pC1Predicted = T_C0_C1Pred.transformPoint(pC0);
		const Eigen::Vector3f pC1          = T_C0_C1.transformPoint(pC0);

		const float dist = (pC1 - pC1Predicted).norm();
		expect(lt(dist, 0.001f));
	};

	return EXIT_SUCCESS;
}

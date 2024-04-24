//
// Created by rokas on 26.07.21.
//

#include "../../include/dv-processing/kinematics/linear_transformer.hpp"

#include "boost/ut.hpp"

#include <Eigen/Geometry>

int main() {
	using namespace boost::ut;
	using namespace dv::kinematics;

	// Test suite build-up
	Eigen::Quaternionf quaternion(1.f, 0.f, 0.f, 0.f);

	size_t capacity = 50;
	LinearTransformerf transformer(capacity);

	transformer.pushTransformation(Transformationf(0, Eigen::Vector3f(0.f, 0.f, 0.f), quaternion));
	transformer.pushTransformation(Transformationf(500, Eigen::Vector3f(1.f, 2.f, 4.f), quaternion));
	transformer.pushTransformation(Transformationf(1000, Eigen::Vector3f(2.f, 4.f, 6.f), quaternion));

	expect(eq(transformer.size(), 3ULL));

	"exact_transform"_test = [&] {
		// Test extraction of first exact transform
		auto T0 = transformer.getTransformAt(0);
		expect(T0.has_value());
		expect(eq(T0->getTranslation()(0), 0.f));
		expect(eq(T0->getTranslation()(1), 0.f));
		expect(eq(T0->getTranslation()(2), 0.f));

		// Last transform extraction
		auto T_final = transformer.getTransformAt(1000);
		expect(T_final.has_value());
		expect(eq(T_final->getTranslation()(0), 2.f));
		expect(eq(T_final->getTranslation()(1), 4.f));
		expect(eq(T_final->getTranslation()(2), 6.f));
	};

	"translation_interpolation"_test = [&] {
		// Interpolate between first and second
		auto T1 = transformer.getTransformAt(250);
		expect(T1.has_value());
		expect(eq(T1->getTimestamp(), 250));
		expect(eq(T1->getTranslation()(0), 0.5f));
		expect(eq(T1->getTranslation()(1), 1.0f));
		expect(eq(T1->getTranslation()(2), 2.0f));
		expect(eq(T1->getQuaternion().w(), 1.0f));

		// Interpolate between second and third
		auto T2 = transformer.getTransformAt(750);
		expect(T2.has_value());
		expect(eq(T2->getTimestamp(), 750));
		expect(eq(T2->getTranslation()(0), 1.5f));
		expect(eq(T2->getTranslation()(1), 3.0f));
		expect(eq(T2->getTranslation()(2), 5.0f));
		expect(eq(T2->getQuaternion().w(), 1.0f));

		// Interpolation at first third
		auto T3 = transformer.getTransformAt(167);
		expect(T3.has_value());
		expect(eq(T3->getTimestamp(), 167));
		expect(lt(T3->getTranslation()(0) - (1.f / 3.f), 0.01f));
		expect(lt(T3->getTranslation()(1) - (2.f / 3.f), 0.01f));
		expect(lt(T3->getTranslation()(2) - (4.f / 3.f), 0.01f));
	};

	"out_of_bound"_test = [&] {
		// Out of bounds tests
		auto T_novalue = transformer.getTransformAt(-1);
		expect(!T_novalue.has_value());
		auto T_novalue2 = transformer.getTransformAt(1001);
		expect(!T_novalue2.has_value());
	};

	"quaternion_interpolation"_test = [&] {
		// Quaternion interpolation test
		Eigen::Quaternionf quaternionAttitude(0.7071f, 0.f, 0.7071f, 0.f);
		transformer.pushTransformation(Transformationf(2000, Eigen::Vector3f(2.f, 4.f, 6.f), quaternionAttitude));
		auto T_halfAttitude = transformer.getTransformAt(1500);
		expect(T_halfAttitude.has_value());
		auto outputQ = T_halfAttitude->getQuaternion();
		expect(le(outputQ.w() - 0.9238844f, std::numeric_limits<float>::epsilon()));
		expect(le(outputQ.x() - 0.f, std::numeric_limits<float>::epsilon()));
		expect(le(outputQ.y() - 0.38627f, std::numeric_limits<float>::epsilon()));
		expect(le(outputQ.z() - 0.f, std::numeric_limits<float>::epsilon()));
	};

	"capacity"_test = [&] {
		// Capacity overflow test
		int64_t timestamp = transformer.latestTransformation().getTimestamp();
		for (size_t i = 0; i < capacity * 2; i++) {
			timestamp += 1000;
			transformer.pushTransformation(Transformationf(timestamp, Eigen::Vector3f(0.f, 0.f, 0.f), quaternion));
		}
		expect(eq(transformer.size(), capacity));
	};

	"out_of_order_exception"_test = [&] {
		expect(throws([&] {
			transformer.pushTransformation(transformer.earliestTransformation());
		})) << "Out of order exception should be thrown";
	};

	"slice_transforms"_test = [&] {
		// Extract part of transforms in the buffer
		int64_t oldestTime    = transformer.earliestTransformation().getTimestamp();
		int64_t latestTime    = transformer.latestTransformation().getTimestamp();
		int64_t thirdOfPeriod = (latestTime - oldestTime) / 3;

		LinearTransformerf slice
			= transformer.getTransformsBetween(oldestTime + thirdOfPeriod, latestTime - thirdOfPeriod);

		expect(le(slice.earliestTransformation().getTimestamp(), oldestTime + thirdOfPeriod));
		expect(ge(slice.latestTransformation().getTimestamp(), latestTime - thirdOfPeriod));
	};

	"resample_transforms"_test = [&] {
		int64_t period
			= transformer.latestTransformation().getTimestamp() - transformer.earliestTransformation().getTimestamp();
		size_t interval              = 1000;
		size_t expectedCount         = (static_cast<unsigned long>(period) / interval) + 1;
		LinearTransformerf resampled = transformer.resampleTransforms(interval);
		expect(eq(resampled.size(), expectedCount));
		expect(
			eq(resampled.earliestTransformation().getTimestamp(), transformer.earliestTransformation().getTimestamp()));
		expect(eq(resampled.latestTransformation().getTimestamp(), transformer.latestTransformation().getTimestamp()));
	};

	"only_one_transform"_test = [] {
		LinearTransformerf transformer(2);
		expect(eq(transformer.size(), 0ULL));

		transformer.pushTransformation(dv::kinematics::Transformationf(10000, Eigen::Matrix4f::Identity()));
		expect(eq(transformer.size(), 1ULL));
		expect(!transformer.getTransformAt(5000).has_value());
		expect(transformer.getTransformAt(10000).has_value());
		expect(!transformer.getTransformAt(15000).has_value());

		transformer.pushTransformation(dv::kinematics::Transformationf(20000, Eigen::Matrix4f::Identity()));
		expect(eq(transformer.size(), 2ULL));
		expect(!transformer.getTransformAt(5000).has_value());
		expect(transformer.getTransformAt(10000).has_value());
		expect(transformer.getTransformAt(15000).has_value());
		expect(transformer.getTransformAt(20000).has_value());
		expect(!transformer.getTransformAt(25000).has_value());

		transformer.pushTransformation(dv::kinematics::Transformationf(30000, Eigen::Matrix4f::Identity()));
		expect(eq(transformer.size(), 2ULL));
		expect(!transformer.getTransformAt(5000).has_value());
		expect(!transformer.getTransformAt(10000).has_value());
		expect(!transformer.getTransformAt(15000).has_value());
		expect(transformer.getTransformAt(20000).has_value());
		expect(transformer.getTransformAt(25000).has_value());
		expect(transformer.getTransformAt(30000).has_value());
		expect(!transformer.getTransformAt(35000).has_value());
	};

	return EXIT_SUCCESS;
}

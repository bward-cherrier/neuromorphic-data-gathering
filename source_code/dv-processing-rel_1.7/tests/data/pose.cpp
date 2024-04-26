//
// Created by rokas on 01.09.21.
//

#include "../../include/dv-processing/data/utilities.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"pose_conversions_identity"_test = [] {
		Eigen::Matrix4f identity;
		identity.setIdentity();

		dv::kinematics::Transformationf transform(100100, identity);

		dv::Pose pose = dv::data::poseFromTransformation(transform);
		expect(eq(pose.timestamp, transform.getTimestamp()));

		expect(eq(pose.translation.x(), 0.f));
		expect(eq(pose.translation.y(), 0.f));
		expect(eq(pose.translation.z(), 0.f));

		expect(eq(pose.rotation.w(), 1.f));
		expect(eq(pose.rotation.x(), 0.f));
		expect(eq(pose.rotation.y(), 0.f));
		expect(eq(pose.rotation.z(), 0.f));

		dv::kinematics::Transformationf backConverted = dv::data::transformFromPose(pose);
		expect(backConverted.getTransform().isApprox(identity));
		expect(eq(backConverted.getTimestamp(), transform.getTimestamp()));
	};

	"pose_conversions"_test = [] {
		Eigen::Matrix4f somerotation;
		somerotation << 0.0000006f, -0.0007963f, 0.9999997f, 0.f, 0.0015927f, -0.9999984f, -0.0007963f, 0.f, 0.9999987f,
			0.0015927f, 0.0000006f, 0.f, 0.f, 0.f, 0.f, 1.f;

		dv::kinematics::Transformationf transform(100100, somerotation);

		dv::Pose pose = dv::data::poseFromTransformation(transform);
		expect(eq(pose.timestamp, transform.getTimestamp()));

		expect(eq(pose.translation.x(), 0.f));
		expect(eq(pose.translation.y(), 0.f));
		expect(eq(pose.translation.z(), 0.f));

		static constexpr float epsilon = std::numeric_limits<float>::epsilon();
		expect(le(pose.rotation.x() - 0.7071065f, epsilon));
		expect(le(pose.rotation.y() - 0.0002815f, epsilon));
		expect(le(pose.rotation.z() - 0.7071065f, epsilon));
		expect(le(pose.rotation.w() - 0.0008446f, epsilon));

		dv::kinematics::Transformationf backConverted = dv::data::transformFromPose(pose);
		expect(backConverted.getTransform().isApprox(somerotation));
		expect(eq(backConverted.getTimestamp(), transform.getTimestamp()));
	};

	return EXIT_SUCCESS;
}

#include <dv-processing/kinematics/linear_transformer.hpp>
#include <dv-processing/kinematics/transformation.hpp>

#include <iostream>

int main() {
	// Declare linear transformer with capacity of 100 transformations. Internally it uses a bounded FIFO queue
	// to manage the transformations.
	dv::kinematics::LinearTransformerf transformer(100);

	// Push first transformation which is an identity matrix, so it starts with no rotation at zero coordinates
	transformer.pushTransformation(
		dv::kinematics::Transformationf(1000000, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity()));

	// Add a second transformation with no rotation as well, but with different translational coordinates
	transformer.pushTransformation(
		dv::kinematics::Transformationf(2000000, Eigen::Vector3f(1.f, 2.f, 3.f), Eigen::Quaternionf::Identity()));

	// Interpolate transformation at a midpoint (time-wise), this should device the translational coordinates
	// by a factor of 2.0
	const auto midpoint = transformer.getTransformAt(1500000);

	// Print the resulting output.
	std::cout << "Interpolated position at [" << midpoint->getTimestamp() << "]: ["
			  << midpoint->getTranslation().transpose() << "]" << std::endl;

	return 0;
}

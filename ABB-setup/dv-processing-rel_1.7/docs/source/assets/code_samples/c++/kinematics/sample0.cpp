#include <dv-processing/kinematics/transformation.hpp>

#include <iostream>

int main() {
	Eigen::Matrix4f matrix;

	// Mirror rotation matrix with 0.5 translational offsets on all axes. The rotation matrix should flip
	// x and z axes of the input.
	matrix << -1.f, 0.f, 0.f, 0.5f, 0.f, 1.f, 0.f, 0.5f, 0.f, 0.f, -1.f, 0.5f, 0.f, 0.f, 0.f, 1.f;

	// Initialize the transformation with the above matrix. The timestamp can be ignored for this sample, so its set
	// to zero.
	const dv::kinematics::Transformationf transformation(0, matrix);

	// Let's take a sample point with offsets of 1 on all axes.
	const Eigen::Vector3f point(1.f, 1.f, 1.f);

	// Apply this transformation to the above point. This should invert x and z axes and add 0.5 to all values.
	const Eigen::Vector3f transformed = transformation.transformPoint(point);

	// Print the resulting output.
	std::cout << "Transformed from [" << point.transpose() << "] to [" << transformed.transpose() << "]" << std::endl;

	return 0;
}

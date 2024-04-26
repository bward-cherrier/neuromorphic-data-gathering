#pragma once

#include "../core/concepts.hpp"

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

namespace dv::kinematics {

/**
 * Basic transformation wrapper containing homogenous 3D transformation and timestamp.
 * @tparam Scalar Customizable storage type - float or double.
 */
template<std::floating_point Scalar>
class Transformation {
private:
	/**
	 * Timestamp of the transformation, Unix timestamp in microseconds.
	 */
	int64_t mTimestamp;

	/**
	 * The transformation itself, stored in 4x4 format:
	 *
	 * R|T
	 * ---
	 * 0|1
	 */
	Eigen::Matrix<Scalar, 4, 4> mT;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Construct the transformation from a timestamp and 4x4 transformation matrix
	 * @param timestamp Unix timestamp in microsecond format
	 * @param T Homogenous 3D transformation matrix
	 */
	Transformation(int64_t timestamp, const Eigen::Matrix<Scalar, 4, 4> &T) : mTimestamp(timestamp), mT(T) {
	}

	/**
	 * Construct an identity transformation from with timestamp.
	 * @param timestamp Unix timestamp in microsecond format
	 */
	Transformation() : mTimestamp(0) {
		mT.setIdentity();
	}

	/**
	 * Construct the transformation from a timestamp and 3x4 non-homogenous transformation matrix.
	 * @param timestamp Unix timestamp in microsecond format
	 * @param T 3x4 3D transformation matrix
	 */
	static Transformation fromNonHomogenous(int64_t timestamp, const Eigen::Matrix<Scalar, 3, 4> &T) {
		Eigen::Matrix<Scalar, 4, 4> transform;
		transform.template block<3, 4>(0, 0) = T;
		transform(3, 0)                      = 0.0;
		transform(3, 1)                      = 0.0;
		transform(3, 2)                      = 0.0;
		transform(3, 3)                      = 1.0;
		return Transformation<Scalar>(timestamp, transform);
	}

	/**
	 * Construct the transformation from timestamp, 3D translation vector and quaternion describing the rotation.
	 * @param timestamp Unix timestamp in microsecond format
	 * @param translation 3D translation vector
	 * @param rotation Quaternion describing the rotation
	 */
	Transformation(
		int64_t timestamp, const Eigen::Matrix<Scalar, 3, 1> &translation, const Eigen::Quaternion<Scalar> &rotation) :
		mTimestamp(timestamp) {
		mT.template block<3, 1>(0, 3) = translation;
		mT.template block<3, 3>(0, 0) = rotation.toRotationMatrix();
		mT(3, 0)                      = 0.0;
		mT(3, 1)                      = 0.0;
		mT(3, 2)                      = 0.0;
		mT(3, 3)                      = 1.0;
	}

	/**
	 * Construct the transformation from timestamp, 3D translation vector and quaternion describing the rotation.
	 * @param timestamp Unix timestamp in microsecond format
	 * @param translation 3D translation vector
	 * @param rotationMatrix Rotation matrix describing the rotation
	 */
	Transformation(int64_t timestamp, const Eigen::Matrix<Scalar, 3, 1> &translation,
		const Eigen::Matrix<Scalar, 3, 3> &rotationMatrix) :
		mTimestamp(timestamp) {
		mT.template block<3, 1>(0, 3) = translation;
		mT.template block<3, 3>(0, 0) = rotationMatrix;
		mT(3, 0)                      = 0.0;
		mT(3, 1)                      = 0.0;
		mT(3, 2)                      = 0.0;
		mT(3, 3)                      = 1.0;
	}

	/**
	 * Construct the transformation from timestamp, 3D translation vector and quaternion describing the rotation.
	 * @param timestamp     Unix timestamp in microsecond format
	 * @param translation   3D translation vector
	 * @param rotation      3x3 rotation matrix
	 */
	Transformation(int64_t timestamp, const cv::Mat &translation, const cv::Mat &rotation) : mTimestamp(timestamp) {
		Eigen::Matrix<Scalar, 3, 1> _translation;
		Eigen::Matrix<Scalar, 3, 3> _rotation;

		// This will handle type casting if needed
		cv::cv2eigen(translation, _translation);
		cv::cv2eigen(rotation, _rotation);

		mT.template block<3, 1>(0, 3) = _translation;
		mT.template block<3, 3>(0, 0) = _rotation;
		mT(3, 0)                      = 0.0;
		mT(3, 1)                      = 0.0;
		mT(3, 2)                      = 0.0;
		mT(3, 3)                      = 1.0;
	}

	/**
	 * Get timestamp.
	 * @return Unix timestamp of the transformation in microseconds.
	 */
	[[nodiscard]] int64_t getTimestamp() const {
		return mTimestamp;
	}

	/**
	 * Get the transformation matrix.
	 * @return Transformation matrix in 4x4 format
	 */
	[[nodiscard]] const Eigen::Matrix<Scalar, 4, 4> &getTransform() const {
		return mT;
	}

	/**
	 * Retrieve a copy of 3x3 rotation matrix.
	 * @return 3x3 rotation matrix
	 */
	[[nodiscard]] inline Eigen::Matrix<Scalar, 3, 3> getRotationMatrix() const {
		return mT.template block<3, 3>(0, 0);
	}

	/**
	 * Retrieve rotation expressed as a quaternion.
	 * @return Quaternion containing rotation.
	 */
	[[nodiscard]] inline Eigen::Quaternion<Scalar> getQuaternion() const {
		return Eigen::Quaternion<Scalar>(getRotationMatrix());
	}

	/**
	 * Retrieve translation as 3D vector.
	 * @return Vector containing translation.
	 */
	template<concepts::Coordinate3DCostructible Output = Eigen::Matrix<Scalar, 3, 1>>
	[[nodiscard]] inline Output getTranslation() const {
		return Output(mT(0, 3), mT(1, 3), mT(2, 3));
	}

	/**
	 * Transform a point using this transformation.
	 * @param point Point to be transformed
	 * @return Transformed point
	 */
	template<concepts::Coordinate3DCostructible Output = Eigen::Matrix<Scalar, 3, 1>, concepts::Coordinate3D Input>
	[[nodiscard]] inline Output transformPoint(const Input &point) const {
		if constexpr (concepts::is_eigen_type<Input>) {
			const Eigen::Matrix<Scalar, 3, 1> transformed = getRotationMatrix() * point + getTranslation();

			return Output(transformed(0, 0), transformed(1, 0), transformed(2, 0));
		}
		else {
			Eigen::Matrix<Scalar, 3, 1> transformed;

			if constexpr (concepts::Coordinate3DAccessors<Input>) {
				transformed = getRotationMatrix() * Eigen::Matrix<Scalar, 3, 1>(point.x(), point.y(), point.z())
							+ getTranslation();
			}
			else {
				transformed
					= getRotationMatrix() * Eigen::Matrix<Scalar, 3, 1>(point.x, point.y, point.z) + getTranslation();
			}

			return Output(transformed(0, 0), transformed(1, 0), transformed(2, 0));
		}
	}

	/**
	 * Apply rotation only transformation on the given point.
	 * @param point Point to be transformed
	 * @return Transformed point
	 */
	template<concepts::Coordinate3DCostructible Output = Eigen::Matrix<Scalar, 3, 1>, concepts::Coordinate3D Input>
	[[nodiscard]] inline Output rotatePoint(const Input &point) const {
		if constexpr (concepts::is_eigen_type<Input>) {
			const Eigen::Matrix<Scalar, 3, 1> transformed = getRotationMatrix() * point;

			return Output(transformed(0, 0), transformed(1, 0), transformed(2, 0));
		}
		else {
			Eigen::Matrix<Scalar, 3, 1> transformed;

			if constexpr (concepts::Coordinate3DAccessors<Input>) {
				transformed = getRotationMatrix() * Eigen::Matrix<Scalar, 3, 1>(point.x(), point.y(), point.z());
			}
			else {
				transformed = getRotationMatrix() * Eigen::Matrix<Scalar, 3, 1>(point.x, point.y, point.z);
			}

			return Output(transformed(0, 0), transformed(1, 0), transformed(2, 0));
		}
	}

	/**
	 * Calculate the inverse homogenous transformation of this transform.
	 * @return Inverse transformation with the current timestamp.
	 */
	[[nodiscard]] inline Transformation<Scalar> inverse() const {
		const auto rotation        = getRotationMatrix();
		const auto translation     = getTranslation();
		const auto rotationInverse = rotation.transpose();
		return Transformation<Scalar>(mTimestamp, -rotationInverse * translation, rotationInverse);
	}

	/**
	 * Find the transformation from current to target.
	 * (T_target_current s.t. p_target = T_target_current*p_current).
	 *
	 * @param target        Target transformation.
	 * @return              Transformation from this to target.
	 */
	[[nodiscard]] Transformation<Scalar> delta(const Transformation<Scalar> &target) const {
		return Transformation<Scalar>(target.getTimestamp(), target.inverse().getTransform() * mT);
	}
};

/**
 * Transformation using single precision float operations
 */
typedef Transformation<float> Transformationf;

/**
 * Transformation using double precision float operations
 */
typedef Transformation<double> Transformationd;

} // namespace dv::kinematics

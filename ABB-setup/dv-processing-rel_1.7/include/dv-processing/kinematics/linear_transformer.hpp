#pragma once

#include "transformation.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/circular_buffer.hpp>

#include <optional>

namespace dv::kinematics {

/**
 * A buffer containing time increasing 3D transformations and capable of timewise linear interpolation
 * between available transforms. Can be used with different underlying floating point types supported
 * by Eigen.
 * @tparam Scalar Underlying floating point number type - float or double.
 */
template<std::floating_point Scalar>
class LinearTransformer {
private:
	using TransformationType = Transformation<Scalar>;
	using TransformationBuffer
		= boost::circular_buffer<TransformationType, Eigen::aligned_allocator<TransformationType>>;

	TransformationBuffer mTransforms;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using iterator       = typename TransformationBuffer::iterator;
	using const_iterator = typename TransformationBuffer::const_iterator;

	explicit LinearTransformer(size_t capacity) : mTransforms(capacity) {
	}

	/**
	 * Push a transformation into the transformation buffer.
	 * @throws logic_error exception when transformation is added out of order.
	 * @param transformation Transformation to be pushed, it must contain increasing timestamp compared
	 * to latest transformation in the buffer, otherwise an exception will be thrown.
	 */
	void pushTransformation(const TransformationType &transformation) {
		if (mTransforms.empty() || transformation.getTimestamp() > mTransforms.back().getTimestamp()) {
			mTransforms.push_back(transformation);
		}
		else {
			throw std::logic_error(
				"Transformation timestamp older than latest, transformation was pushed out of order");
		}
	}

	/**
	 * Generate forward iterator pointing to first transformation in the transformer buffer.
	 * @return 		Buffer start iterator.
	 */
	[[nodiscard]] iterator begin() {
		return mTransforms.begin();
	}

	/**
	 * Generate an iterator representing end of the buffer.
	 * @return 		Buffer end const-iterator.
	 */
	[[nodiscard]] iterator end() {
		return mTransforms.end();
	}

	/**
	 * Generate a const forward iterator pointing to first transformation in the transformer buffer.
	 * @return 		Buffer start const-iterator.
	 */
	[[nodiscard]] const_iterator cbegin() const {
		return mTransforms.cbegin();
	}

	/**
	 * Generate a const iterator representing end of the buffer.
	 * @return 		Buffer end iterator.
	 */
	[[nodiscard]] const_iterator cend() const {
		return mTransforms.cend();
	}

	/**
	 * Delete all transformations from the buffer.
	 */
	inline void clear() {
		mTransforms.clear();
	}

	/**
	 * Check whether the buffer is empty.
	 * @return true if empty, false otherwise
	 */
	[[nodiscard]] inline bool empty() const {
		return mTransforms.empty();
	}

	/**
	 * Get a transform at the given timestamp.
	 *
	 * If no transform with the exact timestamp was pushed, estimates a transform assuming linear motion.
	 * @param timestamp Unix timestamp in microsecond format.
	 * @return Transformation if successful, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<TransformationType> getTransformAt(int64_t timestamp) const {
		if (isWithinTimeRange(timestamp)) {
			auto T_b = bufferLowerBound(timestamp);
			if (T_b->getTimestamp() == timestamp) {
				return *T_b;
			}
			auto T_a = std::prev(T_b);
			if (T_a->getTimestamp() == timestamp) {
				return *T_a;
			}
			auto distance = static_cast<Scalar>(T_b->getTimestamp() - T_a->getTimestamp());
			auto lambda   = static_cast<Scalar>(timestamp - T_a->getTimestamp()) / distance;

			TransformationType interpolated = interpolateComponentwise(*T_a, *T_b, timestamp, lambda);
			return interpolated;
		}
		else {
			return std::nullopt;
		}
	}

	/**
	 * Checks whether the timestamp is within the range of transformations available in the buffer.
	 * @param timestamp Unix microsecond timestamp to be checked.
	 * @return true if the timestamp is within the range of transformations in the buffer.
	 */
	[[nodiscard]] inline bool isWithinTimeRange(int64_t timestamp) const {
		if (mTransforms.empty()) {
			return false;
		}

		if (mTransforms.front().getTimestamp() <= timestamp && mTransforms.back().getTimestamp() >= timestamp) {
			return true;
		}
		return false;
	}

	/**
	 * Return the size of the buffer.
	 * @return Number of transformations available in the buffer.
	 */
	[[nodiscard]] inline size_t size() const {
		return mTransforms.size();
	}

	/**
	 * Return transformation with highest timestamp.
	 * @return Latest transformation in the buffer.
	 */
	[[nodiscard]] inline const TransformationType &latestTransformation() const {
		return mTransforms.back();
	}

	/**
	 * Return transformation with lowest timestamp.
	 * @return Earliest transformation in time available in the buffer.
	 */
	[[nodiscard]] inline const TransformationType &earliestTransformation() const {
		return mTransforms.front();
	}

	/**
	 * Set new capacity, if the size of the buffer is larger than the newCapacity, oldest transformations from the
	 * start will be removed.
	 * @param newCapacity New transformation buffer capacity.
	 */
	inline void setCapacity(size_t newCapacity) {
		mTransforms.rset_capacity(newCapacity);
	}

	/**
	 * Extract transformation between two given timestamps. If timestamps are not at exact available transformations,
	 * additional transformations will be added so the resulting transformer would complete overlap over the period
	 * (if that is possible).
	 * @param start Start Unix timestamp in microseconds.
	 * @param end End Unix timestamp in microseconds.
	 * @return LinearTransformer containing transformations covering the given period.
	 */
	[[nodiscard]] LinearTransformer<Scalar> getTransformsBetween(int64_t start, int64_t end) const {
		LinearTransformer<Scalar> transformer(mTransforms.capacity());
		auto iter = bufferLowerBound(start);
		if (iter != mTransforms.end() && iter != mTransforms.begin()) {
			auto prevIter = std::prev(iter);
			transformer.pushTransformation(*prevIter);
		}
		while (iter != mTransforms.end() && iter->getTimestamp() <= end) {
			transformer.pushTransformation(*iter);
			iter++;
		}
		if (iter != mTransforms.end()) {
			transformer.pushTransformation(*iter);
		}
		if (transformer.empty()) {
			auto enditer = bufferLowerBound(end);
			if (enditer != mTransforms.end()) {
				transformer.pushTransformation(*enditer);
			}
			else {
				transformer.pushTransformation(mTransforms.back());
			}
		}
		return transformer;
	}

	/**
	 * Resample containing transforms into a new transformer, containing interpolated transforms at given interval.
	 * Will contain the last transformation as well, although the interval might not be maintained for the last
	 * transform.
	 * @param samplingInterval Interval in microseconds at which to resample the transformations.
	 * @return Generated transformer with exact capacity of output transformation count.
	 */
	[[nodiscard]] LinearTransformer<Scalar> resampleTransforms(const int64_t samplingInterval) const {
		if (mTransforms.empty()) {
			return LinearTransformer<Scalar>(mTransforms.capacity());
		}

		int64_t start = mTransforms.front().getTimestamp();
		int64_t end   = mTransforms.back().getTimestamp();
		int64_t now   = start;
		LinearTransformer<Scalar> transformer((static_cast<size_t>((end - start) / samplingInterval)) + 2);
		while (now < end) {
			if (auto tf = getTransformAt(now)) {
				transformer.pushTransformation(*tf);
			}
			now += samplingInterval;
		}
		transformer.pushTransformation(mTransforms.back());
		return transformer;
	}

private:
	/**
	 * Perform linear interpolation between two transformations.
	 * @param T_a First transformation.
	 * @param T_b Second transformation.
	 * @param timestamp Interpolated transformation timestamp.
	 * @param lambda Distance point between the two transformation to interpolate.
	 * @return Interpolated transformation.
	 */
	static TransformationType interpolateComponentwise(
		const TransformationType &T_a, const TransformationType &T_b, const int64_t timestamp, Scalar lambda) {
		dv::runtime_assert(
			lambda >= static_cast<Scalar>(0.) && lambda <= static_cast<Scalar>(1.), "lambda value is out of bounds");
		const auto t                            = T_a.getTranslation();
		const Eigen::Matrix<Scalar, 3, 1> t_int = t + lambda * (T_b.getTranslation() - t);
		const Eigen::Quaternion<Scalar> q_int   = T_a.getQuaternion().slerp(lambda, T_b.getQuaternion());

		return TransformationType(timestamp, t_int, q_int);
	}

	/**
	 * Finds the lower bound iterator in the buffer.
	 * @see std::lower_bound
	 * @param t Unix timestamp in microseconds to search for.
	 * @return Iterator to the buffer with timestamp that is *equal or not less* than given timestamp.
	 */
	[[nodiscard]] inline typename TransformationBuffer::const_iterator bufferLowerBound(int64_t t) const {
		return std::lower_bound(
			mTransforms.begin(), mTransforms.end(), t, [](const TransformationType &st, int64_t ts) {
				return st.getTimestamp() < ts;
			});
	}

	/**
	 * Finds the upper bound iterator in the buffer.
	 * @see std::upper_bound
	 * @param t Unix timestamp in microseconds to search for.
	 * @return Iterator to the buffer with timestamp that is *greater* than given timestamp or `end` if not available.
	 */
	[[nodiscard]] inline typename TransformationBuffer::const_iterator bufferUpperBound(int64_t t) const {
		return std::upper_bound(
			mTransforms.begin(), mTransforms.end(), t, [](int64_t ts, const TransformationType &st) {
				return ts < st.getTimestamp();
			});
	}
};

/**
 * LinearTransformer using single precision float operations
 */
typedef LinearTransformer<float> LinearTransformerf;

/**
 * LinearTransformer using double precision float operations
 */
typedef LinearTransformer<double> LinearTransformerd;

} // namespace dv::kinematics

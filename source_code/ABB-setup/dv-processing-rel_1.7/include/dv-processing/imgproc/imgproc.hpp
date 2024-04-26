#pragma once

#include "../external/fmt_compat.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <optional>

namespace dv::imgproc {

/**
 * Conversion from cv::MatStep to Eigen::Stride
 *
 * cv::MatStep stores steps in units of bytes, as the underlying matrix is always stored in uint8_t arrays, which are
 * then interpreted at run-time based on the type (e.g. CV_8U). Contrary to this, Eigen stores matrices in arrays of a
 * type that is determined at compile-time based on a template argument, and therefore stores its strides in units of
 * pointer increments. The conversion between the two can be computed by dividing by or multiplying with sizeof(T).
 *
 * @tparam T the type of the scalars stored in the matrices
 * @param step the step (stride) in the matrix in units of bytes
 * @return the corresponding Eigen::Stride for the cv::MatStep value provided
 */
template<typename T>
[[nodiscard]] inline auto cvMatStepToEigenStride(const cv::MatStep &step) {
	return Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(
		static_cast<long>(step[0] / sizeof(T)), static_cast<long>(step[1] / sizeof(T)));
}

/**
 * Maps an Eigen::Map onto a cv::Mat object. This provides a view to the internal storage of the cv::Mat, it doesn't
 * copy any data.
 *
 * @tparam T the type of the scalars stored in the matrices
 * @param mat the cv::Mat onto which an Eigen::Map should be mapped
 * @return the view into the cv::Mat via an Eigen::Map object
 */
template<typename T>
[[nodiscard]] inline auto cvMatToEigenMap(const cv::Mat &mat) {
	return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0,
		Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>(
		reinterpret_cast<const T *const>(mat.data), mat.rows, mat.cols, cvMatStepToEigenStride<T>(mat.step));
}

/**
 * Maps an Eigen::Map onto a cv::Mat object. This provides a view to the internal storage of the cv::Mat, it doesn't
 * copy any data.
 *
 * @tparam T the type of the scalars stored in the matrices
 * @param mat the cv::Mat onto which an Eigen::Map should be mapped
 * @return the view into the cv::Mat via an Eigen::Map object
 */
template<typename T>
[[nodiscard]] inline auto cvMatToEigenMap(cv::Mat &mat) {
	return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, 0,
		Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>(
		reinterpret_cast<T *>(mat.data), mat.rows, mat.cols, cvMatStepToEigenStride<T>(mat.step));
}

/**
 * Computes the L1 distance between two blocks (patches) of eigen matrices.
 *
 * @tparam T The type of the underlying matrix
 * @param patch1 the first patch
 * @param patch2 the second patch
 * @return the L1 distance between the two patches
 */
template<typename T>
[[nodiscard]] inline auto L1Distance(const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch1,
	const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch2) {
	assert(patch1.rows() == patch2.rows() && patch1.cols() == patch2.cols());

	return (patch1.template cast<double>() - patch2.template cast<double>()).cwiseAbs().sum();
}

/**
 * Computes the L1 distance between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @tparam MAP_OPTIONS The options for the underlying matrix. \see Eigen::Map::MapOptions
 * @tparam STRIDE The stride of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the L1 distance between the two matrices
 */
template<typename T, int32_t MAP_OPTIONS, typename STRIDE>
[[nodiscard]] inline auto L1Distance(
	const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m1, const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return L1Distance(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the L1 distance between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the L1 distance between the two matrices
 */
template<typename T>
[[nodiscard]] inline auto L1Distance(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m1,
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return L1Distance(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the L1 distance between two matrices
 *
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the L1 distance between the two matrices
 */
[[nodiscard]] inline auto L1Distance(const cv::Mat &m1, const cv::Mat &m2) {
	// The size of both matrices must be the same to compute this metric
	assert(m1.size == m2.size);
	// It only makes sense to compute this metric if the matrices have the same number of colour channels
	assert(m1.channels() == 1 && m2.channels() == 1);
	// This implementation assumes the same depth for both matrices to convert to Eigen::Map
	assert(m1.depth() == m2.depth());
	// This implementation assumes the same type for both matrices to convert to Eigen::Map
	assert(m1.type() == m2.type());

	switch (m1.type()) {
		case CV_8U: {
			return L1Distance(cvMatToEigenMap<uint8_t>(m1), cvMatToEigenMap<uint8_t>(m2));
		}
		case CV_8S: {
			return L1Distance(cvMatToEigenMap<int8_t>(m1), cvMatToEigenMap<int8_t>(m2));
		}
		case CV_16U: {
			return L1Distance(cvMatToEigenMap<uint16_t>(m1), cvMatToEigenMap<uint16_t>(m2));
		}
		case CV_16S: {
			return L1Distance(cvMatToEigenMap<int16_t>(m1), cvMatToEigenMap<int16_t>(m2));
		}
		case CV_32S: {
			return L1Distance(cvMatToEigenMap<int32_t>(m1), cvMatToEigenMap<int32_t>(m2));
		}
		default: {
			throw std::invalid_argument(
				fmt::format("Unsupported OpenCV matrix type passed to imgproc::L1Distance: {}", m1.type()));
		}
	}
}

/**
 * Computes the Pearson Correlation between two blocks (patches) of eigen matrices.
 *
 * @tparam T The type of the underlying matrix
 * @param patch1 the first patch
 * @param patch2 the second patch
 * @return the Pearson Correlation between the two patches
 */
template<typename T>
[[nodiscard]] inline auto pearsonCorrelation(const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch1,
	const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch2) {
	assert(patch1.rows() == patch2.rows() && patch1.cols() == patch2.cols());

	if (patch1.rows() == 0 || patch1.cols() == 0) {
		return std::optional(0.0);
	}

	const auto mean1 = patch1.template cast<double>().mean();
	const auto mean2 = patch2.template cast<double>().mean();

	const auto zeroMean1 = (patch1.template cast<double>().array() - mean1);
	const auto zeroMean2 = (patch2.template cast<double>().array() - mean2);

	const auto var1 = zeroMean1.square().sum();
	const auto var2 = zeroMean2.square().sum();

	return (var1 != 0.0 && var2 != 0.0) ? std::optional((zeroMean1 * zeroMean2).sum() / std::sqrt(var1 * var2))
										: std::nullopt;
}

/**
 * Computes the Pearson Correlation between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @tparam MAP_OPTIONS The options for the underlying matrix. \see Eigen::Map::MapOptions
 * @tparam STRIDE The stride of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Pearson Correlation between the two matrices
 */
template<typename T, int32_t MAP_OPTIONS, typename STRIDE>
[[nodiscard]] inline auto pearsonCorrelation(
	const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m1, const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return pearsonCorrelation(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the Pearson Correlation between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Pearson Correlation between the two matrices
 */
template<typename T>
[[nodiscard]] inline auto pearsonCorrelation(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m1,
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return pearsonCorrelation(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the Pearson Correlation between two matrices
 *
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Pearson Correlation between the two matrices
 */
[[nodiscard]] inline auto pearsonCorrelation(const cv::Mat &m1, const cv::Mat &m2) {
	// The size of both matrices must be the same to compute this metric
	assert(m1.size == m2.size);
	// It only makes sense to compute this metric if the matrices have the same number of colour channels
	assert(m1.channels() == 1 && m2.channels() == 1);
	// This implementation assumes the same depth for both matrices to convert to Eigen::Map
	assert(m1.depth() == m2.depth());
	// This implementation assumes the same type for both matrices to convert to Eigen::Map
	assert(m1.type() == m2.type());

	switch (m1.type()) {
		case CV_8U: {
			return pearsonCorrelation(cvMatToEigenMap<uint8_t>(m1), cvMatToEigenMap<uint8_t>(m2));
		}
		case CV_8S: {
			return pearsonCorrelation(cvMatToEigenMap<int8_t>(m1), cvMatToEigenMap<int8_t>(m2));
		}
		case CV_16U: {
			return pearsonCorrelation(cvMatToEigenMap<uint16_t>(m1), cvMatToEigenMap<uint16_t>(m2));
		}
		case CV_16S: {
			return pearsonCorrelation(cvMatToEigenMap<int16_t>(m1), cvMatToEigenMap<int16_t>(m2));
		}
		case CV_32S: {
			return pearsonCorrelation(cvMatToEigenMap<int32_t>(m1), cvMatToEigenMap<int32_t>(m2));
		}
		default: {
			throw std::invalid_argument(
				fmt::format("Unsupported OpenCV matrix type passed to imgproc::pearsonCorrelation: {}", m1.type()));
		}
	}
}

/**
 * Computes the Cosine Distance between two blocks (patches) of eigen matrices.
 *
 * @tparam T The type of the underlying matrix
 * @param patch1 the first patch
 * @param patch2 the second patch
 * @return the Cosine Distance between the two patches
 */
template<typename T>
[[nodiscard]] inline auto cosineDistance(const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch1,
	const Eigen::Block<T, Eigen::Dynamic, Eigen::Dynamic> &patch2) {
	assert(patch1.rows() == patch2.rows() && patch1.cols() == patch2.cols());

	if (patch1.rows() == 0 || patch1.cols() == 0) {
		return std::optional(0.0);
	}

	const auto sqn1 = patch1.template cast<double>().array().square().sum();
	const auto sqn2 = patch2.template cast<double>().array().square().sum();

	return (sqn1 != 0.0 && sqn2 != 0.0)
			 ? std::optional((patch1.template cast<double>().array() * patch2.template cast<double>().array()).sum()
							 / std::sqrt(sqn1 * sqn2))
			 : std::nullopt;
}

/**
 * Computes the Cosine Distance between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Cosine Distance between the two matrices
 */
template<typename T>
[[nodiscard]] inline auto cosineDistance(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m1,
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return cosineDistance(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the Cosine Distance between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @tparam MAP_OPTIONS The options for the underlying matrix. \see Eigen::Map::MapOptions
 * @tparam STRIDE The stride of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Cosine Distance between the two matrices
 */
template<typename T, int32_t MAP_OPTIONS, typename STRIDE>
[[nodiscard]] inline auto cosineDistance(
	const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m1, const Eigen::Map<T, MAP_OPTIONS, STRIDE> &m2) {
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());

	return cosineDistance(m1.block(0, 0, m1.rows(), m1.cols()), m2.block(0, 0, m2.rows(), m2.cols()));
}

/**
 * Computes the Cosine Distance between two matrices
 *
 * @tparam T The type of the underlying matrix
 * @param m1 the first matrix
 * @param m2 the second matrix
 * @return the Cosine Distance between the two matrices
 */
[[nodiscard]] inline auto cosineDistance(const cv::Mat &m1, const cv::Mat &m2) {
	// The size of both matrices must be the same to compute this metric
	assert(m1.size == m2.size);
	// It only makes sense to compute this metric if the matrices have the same number of colour channels
	assert(m1.channels() == 1 && m2.channels() == 1);
	// This implementation assumes the same depth for both matrices to convert to Eigen::Map
	assert(m1.depth() == m2.depth());
	// This implementation assumes the same type for both matrices to convert to Eigen::Map
	assert(m1.type() == m2.type());

	switch (m1.type()) {
		case CV_8U: {
			return cosineDistance(cvMatToEigenMap<uint8_t>(m1), cvMatToEigenMap<uint8_t>(m2));
		}
		case CV_8S: {
			return cosineDistance(cvMatToEigenMap<int8_t>(m1), cvMatToEigenMap<int8_t>(m2));
		}
		case CV_16U: {
			return cosineDistance(cvMatToEigenMap<uint16_t>(m1), cvMatToEigenMap<uint16_t>(m2));
		}
		case CV_16S: {
			return cosineDistance(cvMatToEigenMap<int16_t>(m1), cvMatToEigenMap<int16_t>(m2));
		}
		case CV_32S: {
			return cosineDistance(cvMatToEigenMap<int32_t>(m1), cvMatToEigenMap<int32_t>(m2));
		}
		default: {
			throw std::invalid_argument(
				fmt::format("Unsupported OpenCV matrix type passed to imgproc::cosineDistance: {}", m2.type()));
		}
	}
}

} // namespace dv::imgproc

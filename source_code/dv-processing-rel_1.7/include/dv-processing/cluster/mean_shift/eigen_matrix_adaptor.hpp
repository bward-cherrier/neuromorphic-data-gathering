#pragma once

#include "../../containers/kd_tree.hpp"
#include "kernel.hpp"

#include <Eigen/Dense>

#include <optional>
#include <random>
#include <vector>

namespace dv::cluster::mean_shift {

template<typename TYPE, int32_t ROWS, int32_t COLUMNS, int32_t SAMPLE_ORDER>
class MeanShiftEigenMatrixAdaptor;

/**
 * Convenience alias for n-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic>
using MeanShiftRowMajorMatrixXX = MeanShiftEigenMatrixAdaptor<TYPE, ROWS, COLUMNS, Eigen::RowMajor>;

/**
 * Convenience alias for n-dimensional data in column-major sample order of arbitrary dimensions and number of
 * samples
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic>
using MeanShiftColMajorMatrixXX = MeanShiftEigenMatrixAdaptor<TYPE, ROWS, COLUMNS, Eigen::ColMajor>;

/**
 * Convenience alias for 1-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftRowMajorMatrixX1 = MeanShiftRowMajorMatrixXX<TYPE, SAMPLES, 1>;
/**
 * Convenience alias for3-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftRowMajorMatrixX2 = MeanShiftRowMajorMatrixXX<TYPE, SAMPLES, 2>;

/**
 * Convenience alias for 3-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftRowMajorMatrixX3 = MeanShiftRowMajorMatrixXX<TYPE, SAMPLES, 3>;

/**
 * Convenience alias for 4-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftRowMajorMatrixX4 = MeanShiftRowMajorMatrixXX<TYPE, SAMPLES, 4>;

/**
 * Convenience alias for 1-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftColMajorMatrix1X = MeanShiftColMajorMatrixXX<TYPE, 1, SAMPLES>;

/**
 * Convenience alias for 2-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftColMajorMatrix2X = MeanShiftColMajorMatrixXX<TYPE, 2, SAMPLES>;

/**
 * Convenience alias for 3-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftColMajorMatrix3X = MeanShiftColMajorMatrixXX<TYPE, 3, SAMPLES>;

/**
 * Convenience alias for 4-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using MeanShiftColMajorMatrix4X = MeanShiftColMajorMatrixXX<TYPE, 4, SAMPLES>;

/**
 * @brief This class implements the Mean Shift clustering algorithm.
 *
 * As the Mean Shift algorithm performs a gradient ascent on an estimated probability density function, when applying it
 * to integer data, which has a non-smooth probability density, the quality of the detected clusters depends
 * significantly on the selected bandwidth hyperparameter, as well as the underlying data and the selected kernel.
 * Generally the Gaussian Kernel yields better results for this kind of data, however it comes with a bigger performance
 * impact.
 *
 * The Mean Shift algorithm is an nonparametric estimate of the modes of the underlying probability distribution for
 * the data. It implements an iterative search, starting from points provided by the user, or randomly selected from
 * the data points provided. For each iteration, the current estimate of the mode is replaced by an estimate of the
 * mean value of the surrounding data samples. If the Epanechnikov kernel is used for the underlying density
 * estimate, its so-called "shadow kernel", the flat kernel must be used for the estimate of the mean. This means,
 * that we can simply compute the average value of the data points that lie within a given radius around the current
 * estimate of the mode, and use this as the next estimate. To provide an efficient search for the neighbours of the
 * current mode estimate, a KD tree was used.
 *
 * For the underlying theory, see "The Estimation of the Gradient of a Density Function with
 * Applications in Pattern Recognition" by K. Fukunaga and L. Hostetler as well as "Mean shift, mode seeking, and
 * clustering" by Yizong Cheng.
 *
 * @tparam TYPE the underlying data type
 * @tparam ROWS the number of rows in the data matrix. May be Eigen::Dynamic or >= 0. \see Eigen::Dynamic
 * @tparam COLUMNS the number of columns in the data matrix. May be Eigen::Dynamic or >= 0. \see Eigen::Dynamic
 * @tparam SAMPLE_ORDER the order in which samples are entered in the matrix. \see Eigen::StorageOptions
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic,
	int32_t SAMPLE_ORDER = Eigen::ColMajor>
class MeanShiftEigenMatrixAdaptor {
private:
	// As we would like to have efficient memory accesses, we require that the data is contained in the matrix in the
	// order that Eigen stores them in memory. However, this is not possible for the case of vectors (one dimensional
	// data), as  Eigen doesn't allow row-vectors to be stored in column-major storage, or column-vectors to be stored
	// in row-major storage. Therefore we must differentiate between Eigen's underlying storage order and sample order.
	// Nevertheless, this does not make a difference, as samples in a vector contain only one single element.
	static constexpr int32_t DIMS             = SAMPLE_ORDER == Eigen::ColMajor ? ROWS : COLUMNS;
	static constexpr int32_t NOT_SAMPLE_ORDER = (SAMPLE_ORDER == Eigen::ColMajor ? Eigen::RowMajor : Eigen::ColMajor);
	static constexpr int32_t STORAGE_ORDER    = DIMS == 1 ? NOT_SAMPLE_ORDER : SAMPLE_ORDER;

public:
	using Matrix          = Eigen::Matrix<TYPE, ROWS, COLUMNS, STORAGE_ORDER>;
	using Vector          = Eigen::Matrix<TYPE, SAMPLE_ORDER == Eigen::ColMajor ? ROWS : 1,
        SAMPLE_ORDER == Eigen::ColMajor ? 1 : COLUMNS, STORAGE_ORDER>;
	using VectorOfVectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;

private:
	using ThisType = MeanShiftEigenMatrixAdaptor<TYPE, ROWS, COLUMNS, SAMPLE_ORDER>;

	using KDTree = dv::containers::kd_tree::KDTreeMatrixAdaptor<TYPE, ROWS, COLUMNS, SAMPLE_ORDER>;

	const size_t mNumSamples;

	const size_t mNumDimensions;

	KDTree mData;

	const TYPE mBandwidth;

	const uint32_t mMaxIter;

	const TYPE mConvergence;

	VectorOfVectors mStartingPoints;

public:
	/**
	 * Constructor
	 *
	 * @tparam T The matrix type. Must be of exact same type as MeanShift::Matrix, to avoid copy construction of a
	 * temporary variable and thereby creating dangling references. \see MeanShift::Matrix
	 * @param data The Matrix containing the data. The data is neither copied nor otherwise managed, ownership
	 * remains with the user of this class.
	 * @param bw The bandwidth used for the shift. This is a hyperparameter for the kernel. For the Epanechnikov
	 * kernel this means that all values within a radius of bw are averaged.
	 * @param conv For each starting point, the algorithm is stopped as soon as the absolute value of the shift is
	 * <= conv.
	 * @param maxIter The maximum number of iterations. Detected modes, for which the the number of iterations
	 * exceed this value are not added to the detected clusters.
	 * @param startingPoints Points from which to start the search.
	 * @param numLeaves the maximum number of leaves for the KDTree. \see dv::containers::KDTree
	 */
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftEigenMatrixAdaptor(const T &data, const TYPE bw, TYPE conv, const uint32_t maxIter,
		const VectorOfVectors &startingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.cols()) : static_cast<uint32_t>(data.rows())),
		mNumDimensions(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.rows()) : static_cast<uint32_t>(data.cols())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<TYPE>::epsilon())),
		mStartingPoints(startingPoints) {
	}

	/**
	 * Constructor
	 *
	 * @tparam T The matrix type. Must be of exact same type as MeanShift::Matrix, to avoid copy construction of a
	 * temporary variable and thereby creating dangling references. \see MeanShift::Matrix
	 * @param data The Matrix containing the data. The data is neither copied nor otherwise managed, ownership
	 * remains with the user of this class.
	 * @param bw The bandwidth used for the shift. This is a hyperparameter for the
	 * kernel. For the Epanechnikov kernel this means that all values within a radius of bw are averaged.
	 * @param conv For each starting point, the algorithm is stopped as soon as the absolute value of the shift is
	 * <= conv.
	 * @param maxIter The maximum number of iterations. Detected modes, for which the the number of iterations
	 * exceed this value are not added to the detected clusters.
	 * @param startingPoints Points from which to start the search.
	 * @param numLeaves the maximum number of leaves for the KDTree. \see dv::containers::KDTree
	 */
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftEigenMatrixAdaptor(const T &data, const TYPE bw, TYPE conv, const uint32_t maxIter,
		VectorOfVectors &&startingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.cols()) : static_cast<uint32_t>(data.rows())),
		mNumDimensions(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.rows()) : static_cast<uint32_t>(data.cols())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<TYPE>::epsilon())),
		mStartingPoints(std::move(startingPoints)) {
	}

	/**
	 * Constructor
	 *
	 * @tparam T The matrix type. Must be of exact same type as MeanShift::Matrix, to avoid copy construction of a
	 * temporary variable and thereby creating dangling references. \see MeanShift::Matrix
	 * @param data The Matrix containing the data. The data is neither copied nor otherwise managed, ownership
	 * remains with the user of this class.
	 * @param bw The bandwidth used for the shift. This is a hyperparameter for the kernel. For the Epanechnikov
	 * kernel this means that all values within a radius of bw are averaged.
	 * @param conv For each starting point, the algorithm is stopped as soon as the absolute value of the shift is
	 * <= conv.
	 * @param maxIter The maximum number of iterations. Detected modes, for which the the number of iterations
	 * exceed this value are not added to the detected clusters.
	 * @param numStartingPoints The number of points which are randomly selected from the data points, to be used as
	 * starting points.
	 * @param numLeaves the maximum number of leaves for the KDTree. \see dv::containers::KDTree
	 */
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftEigenMatrixAdaptor(const T &data, const TYPE bw, TYPE conv, const uint32_t maxIter,
		const uint32_t numStartingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.cols()) : static_cast<uint32_t>(data.rows())),
		mNumDimensions(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.rows()) : static_cast<uint32_t>(data.cols())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<TYPE>::epsilon())),
		mStartingPoints(mNumDimensions > 0 && mNumSamples > 0 ? generateStartingPointsFromData(numStartingPoints, data)
															  : VectorOfVectors()) {
	}

	MeanShiftEigenMatrixAdaptor()                                 = delete;
	MeanShiftEigenMatrixAdaptor(const ThisType &other)            = delete;
	MeanShiftEigenMatrixAdaptor(ThisType &&other)                 = delete;
	MeanShiftEigenMatrixAdaptor &operator=(const ThisType &other) = delete;
	MeanShiftEigenMatrixAdaptor &operator=(ThisType &&other)      = delete;
	~MeanShiftEigenMatrixAdaptor()                                = default;

	/**
	 * Executes the algorithm.
	 *
	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @returns The centres of each detected cluster
	 * @returns The labels for each data point. The labels correspond to the index of the centre to which the sample
	 * is assigned.
	 * @returns The number of samples in each cluster
	 * @returns The in-cluster variance for each cluster
	 */
	template<kernel::MeanShiftKernel kernel = kernel::Epanechnikov>
	[[nodiscard]] auto fit() {
		const auto centres                           = findClusterCentres<kernel>();
		const auto [labels, sampleCounts, variances] = assignClusters(centres);

		return std::make_tuple(centres, labels, sampleCounts, variances);
	}

	/**
	 * Generates a vector of vectors containing the starting points by randomly selecting from provided data
	 *
	 * @param numStartingPoints The number of points to be generated
	 * @param data the matrix to select the starting points from
	 * @returns The vector of vectors containing the starting points.
	 */
	[[nodiscard]] static VectorOfVectors generateStartingPointsFromData(
		const uint32_t numStartingPoints, const Matrix &data) {
		VectorOfVectors startingPoints;

		auto numSamples = (SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows());

		const auto indices = randomArrayBetween<uint32_t>(numStartingPoints, 0, static_cast<uint32_t>(numSamples - 1));

		startingPoints.reserve(numStartingPoints);
		for (uint32_t i = 0; i < numStartingPoints; i++) {
			startingPoints.emplace_back(extractSample(data, indices[i]));
		}

		return startingPoints;
	}

	/**
	 * Generates a vector of vectors containing the starting points by generating random points within a given range for
	 * each dimension
	 *
	 * @param numStartingPoints The number of points to be generated
	 * @param ranges a vector containing one range per dimension. Each dimension is represented by a pair containing the
	 * beginning and the end of the range
	 * @returns The vector of vectors containing the starting points.
	 */
	[[nodiscard]] static VectorOfVectors generateStartingPointsFromRange(
		const uint32_t numStartingPoints, const std::vector<std::pair<TYPE, TYPE>> &ranges) {
		VectorOfVectors startingPoints;

		startingPoints.reserve(numStartingPoints);

		std::vector<Eigen::Array<TYPE, 1, Eigen::Dynamic>> randomRangeValues;

		for (const auto &range : ranges) {
			randomRangeValues.emplace_back(randomArrayBetween<TYPE>(numStartingPoints, range.first, range.second));
		}

		for (uint32_t i = 0; i < numStartingPoints; i++) {
			Vector startingPoint = getZeroVector(static_cast<uint32_t>(ranges.size()));

			for (uint8_t j = 0; j < ranges.size(); j++) {
				startingPoint(j) = randomRangeValues[j](i);
			}

			startingPoints.push_back(startingPoint);
		}

		return startingPoints;
	}

private:
	/**
	 * Performs the search for the cluster centres for each given starting point. A detected centre is added to the
	 * set of centres if it isn't closer than the bandwidth to any previously detected centre.
	 *
	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @returns The centres of each detected cluster
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] auto findClusterCentres() {
		VectorOfVectors clusterCentres;

		if (mNumDimensions > 0 && mNumSamples > 0) {
			for (const auto &startingPoint : mStartingPoints) {
				const auto currentMode = performShift<kernel>(startingPoint);

				if (currentMode.has_value() && (!(*currentMode).array().isNaN().any())
					&& std::none_of(
						clusterCentres.begin(), clusterCentres.end(), [this, &currentMode](const auto &centre) {
							return (centre - *currentMode).squaredNorm() < mBandwidth * mBandwidth;
						})) {
					clusterCentres.push_back(*currentMode);
				}
			}
		}

		return clusterCentres;
	}

	/**
	 * Assigns the data samples to a cluster by means of a nearest neighbour search, and computes the number of
	 * samples as well as the in-cluster variance in the process.
	 *
	 * @param clusterCentres The centres of each detected cluster
	 * @returns The labels for each data point. The labels correspond to the index of the centre to which the sample
	 * is assigned.
	 * @returns The number of samples in each cluster
	 * @returns The in-cluster variance for each cluster
	 */
	[[nodiscard]] auto assignClusters(const VectorOfVectors &clusterCentres) {
		std::vector<uint32_t> labels(mNumSamples);
		std::vector<uint32_t> sampleCounts(clusterCentres.size());
		std::vector<TYPE> variances(clusterCentres.size());

		if (!clusterCentres.empty()) {
			for (uint32_t i = 0; i < mNumSamples; i++) {
				const auto sample = getSample(i);

				const auto closestClusterCentre = std::min_element(
					clusterCentres.begin(), clusterCentres.end(), [&sample](const auto &c1, const auto &c2) {
						return (sample - c1).squaredNorm() < (sample - c2).squaredNorm();
					});

				const auto minIndex
					= static_cast<uint32_t>(std::distance(clusterCentres.begin(), closestClusterCentre));
				labels[i] = minIndex;
				sampleCounts[minIndex]++;
				variances[minIndex] += (sample - clusterCentres[minIndex]).squaredNorm();
			}

			for (uint32_t i = 0; i < variances.size(); i++) {
				if (sampleCounts[i] > 1) {
					variances[i] /= static_cast<float>(sampleCounts[i] - 1);
				}
				else {
					variances[i] = 0.0f;
				}
			}
		}

		return std::make_tuple(labels, sampleCounts, variances);
	}

	/**
	 * Performs a search for a mode in the underlying density starting off with a provided initial point.

	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @param currentMode The starting point that is to be shifted until convergence.
	 * @returns An std::optional containing either a vector, if the search has converged, std::nullopt otherwise
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] std::optional<Vector> performShift(Vector currentMode) {
		uint32_t iterations = 0;

		TYPE shift   = 0;
		auto shifted = false;
		float scale;

		do {
			const auto neighbours = getNeighbours<kernel>(currentMode);

			shifted = !neighbours.empty();

			if (shifted) {
				auto shiftedMode = getZeroVector();
				scale            = 0.0f;

				for (const auto &neighbour : neighbours) {
					const auto sample = getSample(static_cast<uint32_t>(neighbour.first));
					const auto sqd    = (currentMode - sample).squaredNorm();

					const auto kernelValue = applyKernel<kernel>(sqd);

					shiftedMode += kernelValue * sample;
					scale       += kernelValue;
				}

				if (scale == 0.0f) {
					// As, by definition, kernels must be non-negative, the scale can only be zero if the kernel was
					// zero for each neighbour. This implies that all of the detected neighbours lie outside of the
					// kernel bandwidth, which means that the current mode must be a mode, as it is the only sample
					// within that bandwidth. Therefore, the search has converged. and the current mode need not be
					// updated.
					shift = 0.0f;
				}
				else {
					shiftedMode /= scale;

					shift       = (currentMode - shiftedMode).squaredNorm();
					currentMode = shiftedMode;
				}
			}
			else {
				shift = 0;
			}

			iterations++;
		}
		while (shifted && (shift > mConvergence) && (iterations < mMaxIter));

		return (shifted && (shift <= mConvergence) && (iterations < mMaxIter)) ? std::optional<Vector>{currentMode}
																			   : std::nullopt;
	}

	/**
	 * Applies the selected kernel to the squared distance
	 *
	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @param squaredDistance the squared distance between the current mode estimate and a given sample point
	 * @return the kernel value
	 */
	template<kernel::MeanShiftKernel kernel>
	float applyKernel(const float squaredDistance) const {
		return kernel::apply(squaredDistance, mBandwidth);
	}

	/**
	 * Returns the neighbours surrounding a centre
	 *
	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @param centre the centre surrounding which the neighbours are to be found
	 * @return the neighbours, as a vector of pairs, one pair per neighbour containing a the index of the point in the
	 * data matrix and a distance to the centre
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] auto getNeighbours(const Vector &currentMode) {
		return mData.radiusSearch(currentMode, kernel::getSearchRadius(mBandwidth));
	}

	/**
	 * Generate an array of random values within a given range and a given length
	 *
	 * @tparam T The data type
	 * @param length The length of the array
	 * @param begin The minimum value contained in the array
	 * @param end The maximum value contained in the array
	 * @return The array
	 */
	template<typename T>
	[[nodiscard]] static auto randomArrayBetween(const uint32_t length, const T begin, const T end) {
		return (((static_cast<float>(end - begin) / 2.0f) * (Eigen::ArrayXf::Random(length) + 1.0f))
				+ static_cast<float>(begin))
			.min(static_cast<float>(end))
			.max(static_cast<float>(begin))
			.template cast<T>();
	}

	/**
	 * Returns a sample at a given index
	 *
	 * @param index the index of the sample in mData
	 * @returns the sample
	 */
	[[nodiscard]] auto getSample(const uint32_t index) const {
		if (mNumSamples > 0 && mNumDimensions > 0) {
			return mData.getSample(index);
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

	/**
	 * Returns a sample at a given index
	 *
	 * @param data the data to extract the sample from
	 * @param index the index of the sample in mData
	 * @returns the sample
	 */
	[[nodiscard]] static auto extractSample(const Matrix &data, const uint32_t index) {
		const auto numSamples    = (SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows());
		const auto numDimensions = (SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols());

		if (numSamples > 0 && numDimensions > 0) {
			if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
				return data.col(index);
			}
			else {
				return data.row(index);
			}
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

	/**
	 * @return a zero vector of length mNumDimensions
	 */
	[[nodiscard]] Vector getZeroVector() const {
		if constexpr (ROWS == Eigen::Dynamic || COLUMNS == Eigen::Dynamic) {
			if constexpr (STORAGE_ORDER == Eigen::ColMajor) {
				return Vector::Zero(mNumDimensions, 1);
			}
			else {
				return Vector::Zero(1, mNumDimensions);
			}
		}
		else {
			return Vector::Zero();
		}
	}

	/**
	 * @return a zero vector of length mNumDimensions
	 */
	[[nodiscard]] static Vector getZeroVector(uint32_t numDimensions) {
		if constexpr (ROWS == Eigen::Dynamic || COLUMNS == Eigen::Dynamic) {
			if constexpr (STORAGE_ORDER == Eigen::ColMajor) {
				return Vector::Zero(numDimensions, 1);
			}
			else {
				return Vector::Zero(1, numDimensions);
			}
		}
		else {
			return Vector::Zero();
		}
	}
};

} // namespace dv::cluster::mean_shift

#pragma once

#include "../../include/dv-processing/cluster/mean_shift.hpp"

#include <Eigen/Dense>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <iostream>
#include <vector>

using Mat = Eigen::Matrix<float, 1, 2>;
BOOST_GEOMETRY_REGISTER_POINT_2D(Mat, float, boost::geometry::cs::cartesian, data()[0], data()[1]);

struct Box {
	const Mat &ll;
	const Mat &ur;
};

BOOST_GEOMETRY_REGISTER_BOX(Box, Mat, ll, ur);

namespace dv::cluster::mean_shift {
/**
 * This class implements the Mean Shift clustering algorithm with an Epanechnikov Kernel
 */
template<typename TYPE, int64_t ROWS = Eigen::Dynamic, int64_t COLUMNS = Eigen::Dynamic,
	int32_t SAMPLE_ORDER = Eigen::ColMajor>
class MeanShiftLinearSearch {
private:
	static constexpr int32_t DIMS = SAMPLE_ORDER == Eigen::ColMajor ? ROWS : COLUMNS;
	static constexpr int32_t STORAGE_ORDER
		= DIMS == 1 ? (SAMPLE_ORDER == Eigen::ColMajor ? Eigen::RowMajor : Eigen::ColMajor) : SAMPLE_ORDER;

public:
	using Matrix          = Eigen::Matrix<TYPE, ROWS, COLUMNS, STORAGE_ORDER>;
	using Vector          = Eigen::Matrix<TYPE, SAMPLE_ORDER == Eigen::ColMajor ? ROWS : 1,
        SAMPLE_ORDER == Eigen::ColMajor ? 1 : COLUMNS, STORAGE_ORDER>;
	using VectorOfVectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;

private:
	static_assert(EIGEN_IMPLIES(ROWS == 1 && COLUMNS != 1, STORAGE_ORDER == Eigen::RowMajor)
				  && "Eigen doesn't allow row-vectors to be stored in column-major storage");

	static_assert(EIGEN_IMPLIES(COLUMNS == 1 && ROWS != 1, STORAGE_ORDER == Eigen::ColMajor)
				  && "Eigen doesn't allow column-vectors to be stored in row-major storage");

	const uint64_t mNumSamples;

	const uint16_t mNumDimensions;

	const Matrix &mData;

	const TYPE mBandwidth;

	const uint64_t mMaxIter;

	const TYPE mConvergence;

	VectorOfVectors mStartingPoints;

	const uint64_t mNumStartingPoints;

public:
	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftLinearSearch(
		const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, const VectorOfVectors &startingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mData(data),
		mBandwidth(bw),
		mStartingPoints(startingPoints),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(mStartingPoints.size()) {
	}

	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftLinearSearch(
		const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, VectorOfVectors &&startingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mData(data),
		mBandwidth(bw),
		mStartingPoints(std::move(startingPoints)),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(mStartingPoints.size()) {
	}

	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftLinearSearch(
		const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, const uint64_t numStartingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mData(data),
		mBandwidth(bw),
		mStartingPoints(generateStartingPoints(numStartingPoints)),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(numStartingPoints) {
	}

	/**
	 * Executes the algorithm
	 */
	[[nodiscard]] auto fit() {
		const auto centres                           = findClusterCentres();
		const auto [labels, sampleCounts, variances] = assignClusters(centres);

		return std::make_tuple(centres, labels, sampleCounts, variances);
	}

private:
	[[nodiscard]] auto findClusterCentres() {
		VectorOfVectors clusterCentres;

		if (mNumSamples > 0 && mNumDimensions > 0) {
			for (uint64_t i = 0; i < mNumStartingPoints; i++) {
				// pick sample i as starting point for mode
				Vector currentMode = getStartingPoint(i);

				// std::cout << "starting point: [" << currentMode << "]" << std::endl;

				// shift mode until convergence
				const bool converged = performShift(currentMode);

				if (converged && !currentMode.array().isNaN().any()) {
					if (std::none_of(
							clusterCentres.begin(), clusterCentres.end(), [this, &currentMode](const auto &centre) {
								return (centre - currentMode).squaredNorm() < mBandwidth * mBandwidth;
							})) {
						clusterCentres.push_back(currentMode);
					}
				}
			}
		}

		return std::move(clusterCentres);
	}

	[[nodiscard]] auto assignClusters(const VectorOfVectors &clusterCentres) {
		std::vector<uint64_t> labels(mNumSamples);
		std::vector<uint64_t> sampleCounts(clusterCentres.size());
		std::vector<TYPE> variances(clusterCentres.size());

		if (clusterCentres.size() > 0) {
			for (uint64_t i = 0; i < mNumSamples; i++) {
				const auto sample = getSample(i);

				const auto closestClusterCentre = std::min_element(
					clusterCentres.begin(), clusterCentres.end(), [&sample](const auto &c1, const auto &c2) {
						return (sample - c1).squaredNorm() < (sample - c2).squaredNorm();
					});

				const auto minIndex = std::distance(clusterCentres.begin(), closestClusterCentre);
				labels[i]           = minIndex;
				sampleCounts[minIndex]++;
				variances[minIndex] += (sample - clusterCentres[minIndex]).squaredNorm();
			}

			for (uint64_t i = 0; i < variances.size(); i++) {
				if (sampleCounts[i] > 1) {
					variances[i] /= (sampleCounts[i] - 1);
				}
				else {
					variances[i] = 0;
				}
			}
		}

		return std::make_tuple(labels, sampleCounts, variances);
	}

	/**
	 * Performs one shift iteration
	 */
	[[nodiscard]] bool performShift(Vector &currentMode) {
		uint64_t iterations = 0;

		auto shiftedMode = getZeroVector();
		float scale;
		TYPE shift   = 0;
		auto shifted = false;
		do {
			shifted     = false;
			shiftedMode = getZeroVector();
			scale       = 0.0;

			for (uint64_t i = 0; i < mNumSamples; i++) {
				const auto currentSample        = getSample(i);
				const auto scaledAndShiftedMode = (currentMode - currentSample) / mBandwidth;
				const auto squaredNorm          = scaledAndShiftedMode.squaredNorm();

				//			std::cout << "currentMode: [" << currentMode << "]" << std::endl
				//					  << "currentSample: [" << currentSample << "]" << std::endl
				//					  << "scaledAndShiftedMode: [" << scaledAndShiftedMode << "]" << std::endl
				//					  << "squaredNorm: " << squaredNorm << std::endl;

				const auto kernelValue = epanechnikovShadowKernel(squaredNorm);

				if (kernelValue > 0.0f) {
					shifted     = true;
					shiftedMode += currentSample * kernelValue;
					scale       += kernelValue;
				}
			}

			if (shifted) {
				shiftedMode /= scale;
				shift       = (currentMode - shiftedMode).squaredNorm();
				currentMode = shiftedMode;
			}
			else {
				shift = 0;
			}

			//		std::cout << "mode: [" << currentMode << "]" << std::endl
			//				  << "shift: " << shift << std::endl
			//				  << "iterations: " << iterations << std::endl;
			iterations++;
		}
		while (shifted && (shift > mConvergence) && (iterations < mMaxIter));

		return (shifted && (shift <= mConvergence) && (iterations < mMaxIter));
	}

	[[nodiscard]] auto generateStartingPoints(const uint64_t numStartingPoints) const {
		VectorOfVectors startingPoints;

		if (mNumSamples > 0 && mNumDimensions > 0) {
			startingPoints.reserve(numStartingPoints);

			const auto indices = randomArrayBetween<int32_t>(numStartingPoints, 0, mNumSamples - 1);

			for (uint64_t i = 0; i < numStartingPoints; i++) {
				startingPoints.emplace_back(getSample(indices[i]));
			}
		}

		return std::move(startingPoints);
	}

	template<typename T>
	auto randomArrayBetween(const uint64_t length, const float begin, const float end) {
		return ((((end - begin) / 2.0f) * (Eigen::ArrayXf::Random(length) + 1.0)) + begin)
			.min(end)
			.max(begin)
			.cast<T>();
	}

	auto epanechnikovShadowKernel(const TYPE squaredShift) {
		// The shadow kernel of the Epanechnikov kernel is the uniform kernel
		// see "Mean Shift, Mode Seeking, and Clustering" by Yizong Cheng
		// and "On Convergence of Epanechnikov Mean Shift" by Kejun Huang et al.
		return squaredShift <= 1.0f ? 1.0f : 0.0f;
	}

	/**
	 * Returns the sample for the index.
	 */
	auto getSample(const uint64_t index) {
		if (mNumSamples > 0 && mNumDimensions > 0) {
			if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
				return mData.col(index);
			}
			else {
				return mData.row(index);
			}
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

	/**
	 * Returns the starting point for the index.
	 */
	[[nodiscard]] auto getStartingPoint(const uint64_t index) const {
		if (mNumSamples > 0 && mNumDimensions > 0) {
			return mStartingPoints[index];
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

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
};

template<typename TYPE, int64_t ROWS = Eigen::Dynamic, int64_t COLUMNS = Eigen::Dynamic,
	int32_t SAMPLE_ORDER = Eigen::ColMajor>
class MeanShiftRTree {
private:
	static constexpr int32_t DIMS = SAMPLE_ORDER == Eigen::ColMajor ? ROWS : COLUMNS;
	static constexpr int32_t STORAGE_ORDER
		= DIMS == 1 ? (SAMPLE_ORDER == Eigen::ColMajor ? Eigen::RowMajor : Eigen::ColMajor) : SAMPLE_ORDER;

public:
	using Matrix          = Eigen::Matrix<TYPE, ROWS, COLUMNS, STORAGE_ORDER>;
	using Vector          = Eigen::Matrix<TYPE, SAMPLE_ORDER == Eigen::ColMajor ? ROWS : 1,
        SAMPLE_ORDER == Eigen::ColMajor ? 1 : COLUMNS, STORAGE_ORDER>;
	using VectorOfVectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;

private:
	static_assert(EIGEN_IMPLIES(ROWS == 1 && COLUMNS != 1, STORAGE_ORDER == Eigen::RowMajor)
				  && "Eigen doesn't allow row-vectors to be stored in column-major storage");

	static_assert(EIGEN_IMPLIES(COLUMNS == 1 && ROWS != 1, STORAGE_ORDER == Eigen::ColMajor)
				  && "Eigen doesn't allow column-vectors to be stored in row-major storage");

	using RTree = boost::geometry::index::rtree<Mat, boost::geometry::index::linear<16>>;

	const uint64_t mNumSamples;

	const uint16_t mNumDimensions;

	RTree mDataTree;

	const Matrix &mDataMat;

	const TYPE mBandwidth;

	const Mat mBandwidthOffsetVector;

	const uint64_t mMaxIter;

	const TYPE mConvergence;

	const VectorOfVectors mStartingPoints;

	const uint64_t mNumStartingPoints;

public:
	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftRTree(
		const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, const VectorOfVectors &startingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mDataTree(),
		mDataMat(data),
		mBandwidth(bw),
		mBandwidthOffsetVector(mBandwidth / 2.0f, mBandwidth / 2.0f),
		mStartingPoints(startingPoints),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(mStartingPoints.size()) {
		if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
			for (uint32_t i = 0; i < data.cols(); i++) {
				mDataTree.insert(data.col(i));
			}
		}
		else {
			for (uint32_t i = 0; i < data.rows(); i++) {
				mDataTree.insert(data.row(i));
			}
		}
	}

	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftRTree(const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, VectorOfVectors &&startingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mDataTree(),
		mDataMat(data),
		mBandwidth(bw),
		mBandwidthOffsetVector(mBandwidth / 2.0f, mBandwidth / 2.0f),
		mStartingPoints(std::move(startingPoints)),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(mStartingPoints.size()) {
		if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
			for (uint32_t i = 0; i < data.cols(); i++) {
				mDataTree.insert(data.col(i));
			}
		}
		else {
			for (uint32_t i = 0; i < data.rows(); i++) {
				mDataTree.insert(data.row(i));
			}
		}
	}

	// Mean shift clustering with Epanechnikov kernel
	// @param data matrix of M x N, M = number of dimensions, N	= number of samples
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	MeanShiftRTree(const T &data, const TYPE bw, TYPE conv, const uint64_t maxIter, const uint64_t numStartingPoints) :
		mNumSamples(SAMPLE_ORDER == Eigen::ColMajor ? data.cols() : data.rows()),
		mNumDimensions(SAMPLE_ORDER == Eigen::ColMajor ? data.rows() : data.cols()),
		mDataTree(),
		mDataMat(data),
		mBandwidth(bw),
		mBandwidthOffsetVector(mBandwidth / 2.0f, mBandwidth / 2.0f),
		mStartingPoints(generateStartingPoints(numStartingPoints)),
		mMaxIter(maxIter),
		mConvergence(conv),
		mNumStartingPoints(numStartingPoints) {
		if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
			for (uint32_t i = 0; i < data.cols(); i++) {
				mDataTree.insert(data.col(i));
			}
		}
		else {
			for (uint32_t i = 0; i < data.rows(); i++) {
				mDataTree.insert(data.row(i));
			}
		}
	}

	/**
	 * Executes the algorithm
	 */
	[[nodiscard]] auto fit() {
		const auto centres                           = findClusterCentres();
		const auto [labels, sampleCounts, variances] = assignClusters(centres);

		return std::make_tuple(centres, labels, sampleCounts, variances);
	}

	[[nodiscard]] auto fitBBox() {
		const auto centres                           = findClusterCentresBBox();
		const auto [labels, sampleCounts, variances] = assignClusters(centres);

		return std::make_tuple(centres, labels, sampleCounts, variances);
	}

private:
	[[nodiscard]] auto findClusterCentres() {
		VectorOfVectors clusterCentres;

		if (mNumSamples > 0 && mNumDimensions > 0) {
			for (uint64_t i = 0; i < mNumStartingPoints; i++) {
				// pick sample i as starting point for mode
				Vector currentMode = getStartingPoint(i);

				// std::cout << "starting point: [" << currentMode << "]" << std::endl;

				// shift mode until convergence
				const bool converged = performShift(currentMode);

				if (converged && !currentMode.array().isNaN().any()) {
					if (std::none_of(
							clusterCentres.begin(), clusterCentres.end(), [this, &currentMode](const auto &centre) {
								return (centre - currentMode).squaredNorm() < mBandwidth * mBandwidth;
							})) {
						clusterCentres.push_back(currentMode);
					}
				}
			}
		}

		return std::move(clusterCentres);
	}

	auto findClusterCentresBBox() {
		VectorOfVectors clusterCentres;

		if (mNumSamples > 0 && mNumDimensions > 0) {
			for (uint64_t i = 0; i < mNumStartingPoints; i++) {
				// pick sample i as starting point for mode
				Vector currentMode = getStartingPoint(i);

				// std::cout << "starting point: [" << currentMode << "]" << std::endl;

				// shift mode until convergence
				const bool converged = performShiftBBox(currentMode);

				if (converged && !currentMode.array().isNaN().any()) {
					if (std::none_of(
							clusterCentres.begin(), clusterCentres.end(), [this, &currentMode](const auto &centre) {
								return (centre - currentMode).squaredNorm() < mBandwidth * mBandwidth;
							})) {
						clusterCentres.push_back(currentMode);
					}
				}
			}
		}

		return std::move(clusterCentres);
	}

	[[nodiscard]] auto assignClusters(const VectorOfVectors &clusterCentres) {
		std::vector<uint64_t> labels(mNumSamples);
		std::vector<uint64_t> sampleCounts(clusterCentres.size());
		std::vector<TYPE> variances(clusterCentres.size());

		if (clusterCentres.size() > 0) {
			for (uint64_t i = 0; i < mNumSamples; i++) {
				const auto sample = getSample(i);

				const auto closestClusterCentre = std::min_element(
					clusterCentres.begin(), clusterCentres.end(), [&sample](const auto &c1, const auto &c2) {
						return (sample - c1).squaredNorm() < (sample - c2).squaredNorm();
					});

				const auto minIndex = std::distance(clusterCentres.begin(), closestClusterCentre);
				labels[i]           = minIndex;
				sampleCounts[minIndex]++;
				variances[minIndex] += (sample - clusterCentres[minIndex]).squaredNorm();
			}

			for (uint64_t i = 0; i < variances.size(); i++) {
				if (sampleCounts[i] > 1) {
					variances[i] /= (sampleCounts[i] - 1);
				}
				else {
					variances[i] = 0;
				}
			}
		}

		return std::make_tuple(labels, sampleCounts, variances);
	}

	/**
	 * Performs one shift iteration
	 */
	bool performShift(Vector &currentMode) {
		uint64_t iterations = 0;

		auto shiftedMode = getZeroVector();
		TYPE shift       = 0;
		auto shifted     = false;
		do {
			shiftedMode = getZeroVector();

			std::vector<Matrix> neighbours;
			mDataTree.query(boost::geometry::index::satisfies([this, &currentMode](Mat const &v) {
				return (v - currentMode).squaredNorm() < mBandwidth * mBandwidth;
			}),
				std::back_inserter(neighbours));

			const auto numNeighbours = neighbours.size();
			shifted                  = numNeighbours > 0;

			if (shifted) {
				shiftedMode = std::accumulate(neighbours.begin(), neighbours.end(), getZeroVector(),
								  [this](const auto &previous, const auto &neighbour) {
									  return previous + neighbour;
								  })
							/ static_cast<float>(numNeighbours);
				shift       = (currentMode - shiftedMode).squaredNorm();
				currentMode = shiftedMode;
			}
			else {
				shift = 0;
			}

			//		std::cout << "mode: [" << currentMode << "]" << std::endl
			//				  << "shift: " << shift << std::endl
			//				  << "iterations: " << iterations << std::endl;
			iterations++;
		}
		while (shifted && (shift > mConvergence) && (iterations < mMaxIter));

		return (shifted && (shift <= mConvergence) && (iterations < mMaxIter));
	}

	bool performShiftBBox(Vector &currentMode) {
		uint64_t iterations = 0;

		auto shiftedMode = getZeroVector();
		float scale;
		TYPE shift   = 0;
		auto shifted = false;
		do {
			shifted     = false;
			shiftedMode = getZeroVector();
			scale       = 0.0;

			std::vector<Matrix> neighbours;

			const Box bbox = {currentMode - mBandwidthOffsetVector, currentMode + mBandwidthOffsetVector};
			mDataTree.query(boost::geometry::index::within(bbox), std::back_inserter(neighbours));

			for (const auto &neighbour : neighbours) {
				const auto scaledAndShiftedMode = (currentMode - neighbour) / mBandwidth;
				const auto squaredNorm          = scaledAndShiftedMode.squaredNorm();

				//			std::cout << "currentMode: [" << currentMode << "]" << std::endl
				//					  << "currentSample: [" << currentSample << "]" << std::endl
				//					  << "scaledAndShiftedMode: [" << scaledAndShiftedMode << "]" << std::endl
				//					  << "squaredNorm: " << squaredNorm << std::endl;

				const auto kernelValue = epanechnikovShadowKernel(squaredNorm);

				if (kernelValue > 0.0f) {
					shifted     = true;
					shiftedMode += neighbour * kernelValue;
					scale       += kernelValue;
				}
			}

			if (shifted) {
				shiftedMode /= scale;
				shift       = (currentMode - shiftedMode).squaredNorm();
				currentMode = shiftedMode;
			}
			else {
				shift = 0;
			}

			//		std::cout << "mode: [" << currentMode << "]" << std::endl
			//				  << "shift: " << shift << std::endl
			//				  << "iterations: " << iterations << std::endl;
			iterations++;
		}
		while (shifted && (shift > mConvergence) && (iterations < mMaxIter));

		return (shifted && (shift <= mConvergence) && (iterations < mMaxIter));
	}

	[[nodiscard]] auto generateStartingPoints(const uint64_t numStartingPoints) const {
		VectorOfVectors startingPoints;

		if (mNumSamples > 0 && mNumDimensions > 0) {
			startingPoints.reserve(numStartingPoints);

			const auto indices = randomArrayBetween<int32_t>(numStartingPoints, 0, mNumSamples - 1);

			for (uint64_t i = 0; i < numStartingPoints; i++) {
				startingPoints.emplace_back(getSample(indices[i]));
			}
		}

		return std::move(startingPoints);
	}

	template<typename T>
	auto randomArrayBetween(const uint64_t length, const float begin, const float end) {
		return ((((end - begin) / 2.0f) * (Eigen::ArrayXf::Random(length) + 1.0)) + begin)
			.min(end)
			.max(begin)
			.cast<T>();
	}

	auto epanechnikovShadowKernel(const TYPE squaredShift) {
		// The shadow kernel of the Epanechnikov kernel is the uniform kernel
		// see "Mean Shift, Mode Seeking, and Clustering" by Yizong Cheng
		// and "On Convergence of Epanechnikov Mean Shift" by Kejun Huang et al.
		return squaredShift <= 1.0f ? 1.0f : 0.0f;
	}

	/**
	 * Returns the sample for the index.
	 */
	[[nodiscard]] auto getSample(const uint64_t index) {
		if (mNumSamples > 0 && mNumDimensions > 0) {
			if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
				return mDataMat.col(index);
			}
			else {
				return mDataMat.row(index);
			}
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

	[[nodiscard]] auto getStartingPoint(const uint64_t index) const {
		if (mNumSamples > 0 && mNumDimensions > 0) {
			return mStartingPoints[index];
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}

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
};
} // namespace dv::cluster::mean_shift

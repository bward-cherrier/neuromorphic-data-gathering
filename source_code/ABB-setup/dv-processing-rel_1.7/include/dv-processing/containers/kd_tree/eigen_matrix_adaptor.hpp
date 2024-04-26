#pragma once

#include "../../external/nanoflann/nanoflann.hpp"

#include <Eigen/Dense>

#include <memory>

namespace dv::containers::kd_tree {

template<typename TYPE, int32_t ROWS, int32_t COLUMNS, int32_t SAMPLE_ORDER>
class KDTreeMatrixAdaptor;

/**
 * Convenience alias for n-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic>
using KDTreeRowMajorXX = KDTreeMatrixAdaptor<TYPE, ROWS, COLUMNS, Eigen::RowMajor>;

/**
 * Convenience alias for n-dimensional data in column-major sample order of arbitrary dimensions and number of samples
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic>
using KDTreeColMajorXX = KDTreeMatrixAdaptor<TYPE, ROWS, COLUMNS, Eigen::ColMajor>;

/**
 * Convenience alias for 1-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeRowMajorX1 = KDTreeRowMajorXX<TYPE, SAMPLES, 1>;

/**
 * Convenience alias for 2-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeRowMajorX2 = KDTreeRowMajorXX<TYPE, SAMPLES, 2>;

/**
 * Convenience alias for 3-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeRowMajorX3 = KDTreeRowMajorXX<TYPE, SAMPLES, 3>;

/**
 * Convenience alias for 4-dimensional data in row-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeRowMajorX4 = KDTreeRowMajorXX<TYPE, SAMPLES, 4>;

/**
 * Convenience alias for 1-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeColMajor1X = KDTreeColMajorXX<TYPE, 1, SAMPLES>;

/**
 * Convenience alias for 2-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeColMajor2X = KDTreeColMajorXX<TYPE, 2, SAMPLES>;

/**
 * Convenience alias for 3-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeColMajor3X = KDTreeColMajorXX<TYPE, 3, SAMPLES>;

/**
 * Convenience alias for 5-dimensional data in column-major sample order of arbitrary number of samples
 */
template<typename TYPE, int32_t SAMPLES = Eigen::Dynamic>
using KDTreeColMajor4X = KDTreeColMajorXX<TYPE, 4, SAMPLES>;

/**
 * Wrapper class around nanoflann::KDTree for data contained in Eigen matrices, which provides efficient approximate
 * nearest neighbour search as well as radius search.
 *
 * @tparam TYPE the underlying data type
 * @tparam ROWS the number of rows in the data matrix. May be Eigen::Dynamic or >= 0. \see Eigen::Dynamic
 * @tparam COLUMNS the number of columns in the data matrix. May be Eigen::Dynamic or >= 0. \see Eigen::Dynamic
 * @tparam SAMPLE_ORDER the order in which samples are entered in the matrix. \see Eigen::StorageOptions
 */
template<typename TYPE, int32_t ROWS = Eigen::Dynamic, int32_t COLUMNS = Eigen::Dynamic,
	int32_t SAMPLE_ORDER = Eigen::ColMajor>
class KDTreeMatrixAdaptor {
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
	using Matrix = Eigen::Matrix<TYPE, ROWS, COLUMNS, STORAGE_ORDER>;
	using Vector = Eigen::Matrix<TYPE, SAMPLE_ORDER == Eigen::ColMajor ? ROWS : 1,
		SAMPLE_ORDER == Eigen::ColMajor ? 1 : COLUMNS, STORAGE_ORDER>;

private:
	using ThisType = KDTreeMatrixAdaptor<TYPE, ROWS, COLUMNS, SAMPLE_ORDER>;
	using Tree     = nanoflann::KDTreeEigenMatrixAdaptor<Matrix, SAMPLE_ORDER == Eigen::ColMajor ? ROWS : COLUMNS,
        nanoflann::metric_L2_Simple, SAMPLE_ORDER == Eigen::RowMajor>;

	static_assert(EIGEN_IMPLIES(ROWS == 1 && COLUMNS != 1, STORAGE_ORDER == Eigen::RowMajor)
				  && "Eigen doesn't allow row-vectors to be stored in column-major storage");

	static_assert(EIGEN_IMPLIES(COLUMNS == 1 && ROWS != 1, STORAGE_ORDER == Eigen::ColMajor)
				  && "Eigen doesn't allow column-vectors to be stored in row-major storage");

	const uint32_t mNumSamples;

	const uint32_t mNumDimensions;

	std::unique_ptr<Tree> mTree;

public:
	/**
	 * Constructor
	 *
	 * @tparam T The matrix type. Must be of exact same type as MeanShift::Matrix, to avoid copy construction of a
	 * temporary variable and thereby creating dangling references. \see MeanShift::Matrix
	 * @param data The Matrix containing the data. The data is neither copied nor otherwise managed, ownership remains
	 * with the user of this class.
	 * @param maxLeaves the maximum number of leaves for the KDTree. A smaller number typically increases the time used
	 * for construction of the tree, but may decrease the time used for searching it. A higher number typically does the
	 * opposite.
	 */
	template<typename T, std::enable_if_t<std::is_same_v<T, Matrix>, bool> = false>
	explicit KDTreeMatrixAdaptor(const T &data, const uint32_t maxLeaves = 32768) :
		mNumSamples(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.cols()) : static_cast<uint32_t>(data.rows())),
		mNumDimensions(
			SAMPLE_ORDER == Eigen::ColMajor ? static_cast<uint32_t>(data.rows()) : static_cast<uint32_t>(data.cols())),
		mTree((mNumSamples > 0) && (mNumDimensions > 0) ? std::make_unique<Tree>(mNumDimensions, data, maxLeaves)
														: nullptr) {
	}

	KDTreeMatrixAdaptor()                                 = delete;
	KDTreeMatrixAdaptor(const ThisType &other)            = delete;
	KDTreeMatrixAdaptor(ThisType &&other)                 = delete;
	KDTreeMatrixAdaptor &operator=(const ThisType &other) = delete;
	KDTreeMatrixAdaptor &operator=(ThisType &&other)      = delete;
	~KDTreeMatrixAdaptor()                                = default;

	/**
	 * Searches for the k nearest neighbours surrounding centrePoint.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param numClosest The number of neighbours to be searched (i.e. the parameter "k")
	 * @return A pair containing the indices of the neighbours in the underlying matrix as well as the distances to
	 * centrePoint
	 */
	auto knnSearch(const Vector &centrePoint, const size_t numClosest) const {
		std::vector<std::pair<typename Matrix::Index, TYPE>> indicesAndDistances;

		if (mTree != nullptr) {
			indicesAndDistances.resize(numClosest);

			const auto numNeighbours = mTree->index->knnSearch(centrePoint.data(), numClosest, indicesAndDistances);

			if (numNeighbours < numClosest) {
				indicesAndDistances.resize(numNeighbours);
			}
		}

		return indicesAndDistances;
	}

	/**
	 * Searches for all neighbours surrounding centrePoint that are within a certain radius.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param radius The radius
	 * @param eps The search accuracy
	 * @param sorted True if the neighbours should be sorted with respect to their distance to centrePoint (comes with a
	 * significant performance impact)
	 * @return A vector of pairs containing the indices of the neighbours in the underlying matrix as well as the
	 * distances to centrePoint
	 */
	auto radiusSearch(const Vector &centrePoint, const TYPE &radius, float eps = 0.0f, bool sorted = false) const {
		const auto params = nanoflann::SearchParams(0, eps, sorted);
		std::vector<std::pair<typename Matrix::Index, TYPE>> indicesAndDistances;
		indicesAndDistances.reserve(100000);

		if (mTree != nullptr) {
			mTree->index->radiusSearch(centrePoint.data(), radius, indicesAndDistances, params);
		}

		indicesAndDistances.shrink_to_fit();

		return indicesAndDistances;
	}

	/**
	 * Returns a sample at a given index
	 *
	 * @param index the index of the sample in mData
	 * @returns the sample
	 */
	[[nodiscard]] auto getSample(const uint32_t index) const {
		if (mTree != nullptr) {
			if constexpr (SAMPLE_ORDER == Eigen::ColMajor) {
				return mTree->m_data_matrix.get().col(index);
			}
			else {
				return mTree->m_data_matrix.get().row(index);
			}
		}
		else {
			throw std::runtime_error("There are no samples in the data matrix.");
		}
	}
};

} // namespace dv::containers::kd_tree

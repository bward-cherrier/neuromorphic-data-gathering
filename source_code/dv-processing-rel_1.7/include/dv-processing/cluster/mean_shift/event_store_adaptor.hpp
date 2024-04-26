#pragma once

#include "../../containers/kd_tree.hpp"
#include "kernel.hpp"

#include <optional>
#include <random>
#include <vector>

namespace dv::cluster::mean_shift {

/**
 * @brief This class implements the Mean Shift clustering algorithm with an Epanechnikov Kernel for event store data.
 *
 * As event data has a non-smooth probability density in x and y space, and the Mean Shift algorithm performs a gradient
 * ascent, the quality of the detected clusters depends significantly on the selected bandwidth hyperparameter, as well
 * as the underlying data and the selected kernel. Generally the Gaussian Kernel yields better results for this kind of
 * data, however it comes with a bigger performance impact.
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
 */
class MeanShiftEventStoreAdaptor {
public:
	using Vector          = dv::TimedKeyPoint;
	using VectorOfVectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;

private:
	using KDTree = dv::containers::kd_tree::KDTreeEventStoreAdaptor;

	const size_t mNumSamples;

	KDTree mData;

	const int16_t mBandwidth;

	const uint32_t mMaxIter;

	const float mConvergence;

	const VectorOfVectors mStartingPoints;

public:
	/**
	 * Constructor
	 *
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
	MeanShiftEventStoreAdaptor(const dv::EventStore &data, const int16_t bw, float conv, const uint32_t maxIter,
		const VectorOfVectors &startingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(static_cast<uint32_t>(data.size())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<float>::epsilon())),
		mStartingPoints(startingPoints) {
	}

	/**
	 * Constructor
	 *
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
	MeanShiftEventStoreAdaptor(const dv::EventStore &data, const int16_t bw, float conv, const uint32_t maxIter,
		VectorOfVectors &&startingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(static_cast<uint32_t>(data.size())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<float>::epsilon())),
		mStartingPoints(std::move(startingPoints)) {
	}

	/**
	 * Constructor
	 *
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
	MeanShiftEventStoreAdaptor(const dv::EventStore &data, const int16_t bw, float conv, const uint32_t maxIter,
		const uint32_t numStartingPoints, const uint32_t numLeaves = 32768) :
		mNumSamples(static_cast<uint32_t>(data.size())),
		mData(data, numLeaves),
		mBandwidth(bw),
		mMaxIter(maxIter),
		mConvergence(std::max(conv, std::numeric_limits<float>::epsilon())),
		mStartingPoints(data.size() > 0 ? generateStartingPointsFromData(numStartingPoints, data) : VectorOfVectors()) {
	}

	MeanShiftEventStoreAdaptor()                                                   = delete;
	MeanShiftEventStoreAdaptor(const MeanShiftEventStoreAdaptor &other)            = delete;
	MeanShiftEventStoreAdaptor(MeanShiftEventStoreAdaptor &&other)                 = delete;
	MeanShiftEventStoreAdaptor &operator=(const MeanShiftEventStoreAdaptor &other) = delete;
	MeanShiftEventStoreAdaptor &operator=(MeanShiftEventStoreAdaptor &&other)      = delete;
	~MeanShiftEventStoreAdaptor()                                                  = default;

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
		const uint32_t numStartingPoints, const dv::EventStore &data) {
		VectorOfVectors startingPoints;

		startingPoints.reserve(numStartingPoints);

		std::default_random_engine generator;
		std::uniform_int_distribution<uint32_t> index(0, static_cast<uint32_t>(data.size() - 1));

		for (uint32_t i = 0; i < numStartingPoints; i++) {
			const auto sample = data[index(generator)];
			startingPoints.emplace_back(dv::Point2f(sample.x(), sample.y()), 0.0f, 0.0f, 0.0f, 0, 0, 0);
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
		const uint32_t numStartingPoints, const std::array<std::pair<int16_t, int16_t>, 2> &ranges) {
		VectorOfVectors startingPoints;

		startingPoints.reserve(numStartingPoints);

		std::default_random_engine generator;
		std::uniform_int_distribution<int16_t> x(ranges[0].first, ranges[0].second);
		std::uniform_int_distribution<int16_t> y(ranges[1].first, ranges[1].second);

		for (uint32_t i = 0; i < numStartingPoints; i++) {
			startingPoints.emplace_back(dv::Point2f(x(generator), y(generator)), 0.0f, 0.0f, 0.0f, 0, 0, 0);
		}

		return startingPoints;
	}

	/**
	 * Performs the search for the cluster centres for each given starting point. A detected centre is added to the
	 * set of centres if it isn't closer than the bandwidth to any previously detected centre.

	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @returns The centres of each detected cluster
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] VectorOfVectors findClusterCentres() {
		VectorOfVectors clusterCentres;

		if (mNumSamples > 0) {
			for (const auto &startingPoint : mStartingPoints) {
				const auto mode = performShift<kernel>(startingPoint);

				if (mode.has_value()
					&& std::none_of(clusterCentres.begin(), clusterCentres.end(), [this, &mode](const auto &centre) {
						   return squaredDistance(centre, *mode) < mBandwidth * mBandwidth;
					   })) {
					clusterCentres.push_back(*mode);
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
	[[nodiscard]] std::tuple<std::vector<uint32_t>, std::vector<uint32_t>, std::vector<float>> assignClusters(
		const VectorOfVectors &clusterCentres) {
		std::vector<uint32_t> labels(mNumSamples);
		std::vector<uint32_t> sampleCounts(clusterCentres.size());
		std::vector<float> variances(clusterCentres.size());

		if (!clusterCentres.empty()) {
			uint32_t sampleIndex = 0;
			for (const auto &sample : mData) {
				const auto closestClusterCentre = std::min_element(
					clusterCentres.begin(), clusterCentres.end(), [this, &sample](const auto &c1, const auto &c2) {
						return squaredDistance(c1, sample) < squaredDistance(c2, sample);
					});

				const auto minIndex
					= static_cast<uint32_t>(std::distance(clusterCentres.begin(), closestClusterCentre));
				labels[sampleIndex] = minIndex;
				sampleCounts[minIndex]++;
				variances[minIndex] += squaredDistance(clusterCentres[minIndex], sample);
				sampleIndex++;
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

private:
	/**
	 * Performs a search for a mode in the underlying density starting off with a provided initial point.
	 *
	 * @tparam kernel the kernel to be used. \see MeanShiftKernel
	 * @param currentMode The starting point that is to be shifted until convergence.
	 * @returns An std::optional containing either a vector, if the search has converged, std::nullopt otherwise
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] std::optional<Vector> performShift(Vector currentMode) {
		uint32_t iterations = 0;

		float shift  = 0.0f;
		auto shifted = false;
		float scale;

		do {
			const auto neighbours = getNeighbours<kernel>(currentMode);

			shifted = !neighbours.empty();

			if (shifted) {
				auto shiftedMode = getZeroVector();
				scale            = 0.0f;

				for (const auto &neighbour : neighbours) {
					const auto *sample = neighbour.first;
					const auto sqd     = squaredDistance(currentMode, *sample);

					const auto kernelValue = applyKernel<kernel>(sqd);

					shiftedMode.pt = dv::Point2f(shiftedMode.pt.x() + (kernelValue * static_cast<float>(sample->x())),
						shiftedMode.pt.y() + (kernelValue * static_cast<float>(sample->y())));
					scale          += kernelValue;
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
					shiftedMode.pt = dv::Point2f(shiftedMode.pt.x() / scale, shiftedMode.pt.y() / scale);

					shift       = squaredDistance(currentMode, shiftedMode);
					currentMode = shiftedMode;
				}
			}
			else {
				shift = 0.0f;
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
	 * @return the neighbours, as a vector of pairs, one pair per neighbour containing a pointer to the event and a
	 * distance to the centre
	 */
	template<kernel::MeanShiftKernel kernel>
	[[nodiscard]] auto getNeighbours(const Vector &centre) {
		return mData.radiusSearch(centre, kernel::getSearchRadius(mBandwidth));
	}

	[[nodiscard]] float squaredDistance(const dv::TimedKeyPoint &k, const dv::Event &e) const {
		return pow2(k.pt.x() - static_cast<float>(e.x())) + pow2(k.pt.y() - static_cast<float>(e.y()));
	}

	[[nodiscard]] float squaredDistance(const dv::TimedKeyPoint &k1, const dv::TimedKeyPoint &k2) const {
		return pow2(k1.pt.x() - k2.pt.x()) + pow2(k1.pt.y() - k2.pt.y());
	}

	[[nodiscard]] float squaredDistance(const dv::Event &e1, const dv::Event &e2) const {
		return static_cast<float>(pow2(e1.x() - e2.x()) + pow2(e1.y() - e2.y()));
	}

	template<typename T>
	[[nodiscard]] T pow2(const T val) const {
		return val * val;
	}

	[[nodiscard]] static Vector getZeroVector() {
		return {dv::Point2f(0, 0), 0.0f, 0.0f, 0.0f, 0, 0, 0};
	}
};

} // namespace dv::cluster::mean_shift

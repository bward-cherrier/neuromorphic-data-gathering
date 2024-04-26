#pragma once

#include "../../external/nanoflann/nanoflann.hpp"

#include "../../core/core.hpp"
#include "../../data/timed_keypoint_base.hpp"

#include <opencv2/core.hpp>

#include <memory>

namespace dv::containers::kd_tree {
/**
 * Wrapper class around nanoflann::KDTree for dv::EventStore data, which provides efficient approximate nearest
 * neighbour search as well as radius search.
 */
class KDTreeEventStoreAdaptor {
private:
	using Index = nanoflann::KDTreeSingleIndexNonContiguousIteratorAdaptor<
		nanoflann::metric_L2_Simple::traits<int32_t, KDTreeEventStoreAdaptor, const dv::Event *>::distance_t,
		KDTreeEventStoreAdaptor, 2, const dv::Event *>;

	const dv::EventStore &mData;
	std::unique_ptr<Index> mIndex;

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
	KDTreeEventStoreAdaptor(const dv::EventStore &data, const uint32_t maxLeaves = 32768) :
		mData(data),
		mIndex(data.size() > 0 ? std::make_unique<Index>(2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaves))
							   : nullptr) {
	}

	KDTreeEventStoreAdaptor()                                                = delete;
	KDTreeEventStoreAdaptor(const KDTreeEventStoreAdaptor &other)            = delete;
	KDTreeEventStoreAdaptor(KDTreeEventStoreAdaptor &&other)                 = delete;
	KDTreeEventStoreAdaptor &operator=(const KDTreeEventStoreAdaptor &other) = delete;
	KDTreeEventStoreAdaptor &operator=(KDTreeEventStoreAdaptor &&other)      = delete;
	~KDTreeEventStoreAdaptor()                                               = default;

	/**
	 * Searches for the k nearest neighbours surrounding centrePoint.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param numClosest The number of neighbours to be searched (i.e. the parameter "k")
	 * @return The number of actually found neighbours
	 */
	template<typename T>
	[[nodiscard]] auto knnSearch(const cv::Point_<T> &centrePoint, const size_t numClosest) const {
		return knnSearch(centrePoint.x, centrePoint.y, numClosest);
	}

	/**
	 * Searches for the k nearest neighbours surrounding centrePoint.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param numClosest The number of neighbours to be searched (i.e. the parameter "k")
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] auto knnSearch(const dv::Event &centrePoint, const size_t numClosest) const {
		return knnSearch(centrePoint.x(), centrePoint.y(), numClosest);
	}

	/**
	 * Searches for the k nearest neighbours surrounding centrePoint.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param numClosest The number of neighbours to be searched (i.e. the parameter "k")
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] auto knnSearch(const dv::TimedKeyPoint &centrePoint, const size_t numClosest) const {
		return knnSearch(static_cast<int32_t>(std::round(centrePoint.pt.x())),
			static_cast<int32_t>(std::round(centrePoint.pt.y())), numClosest);
	}

	/**
	 * Searches for the k nearest neighbours surrounding centrePoint.
	 *
	 * @param x The x-coordinate of the centre point for which the nearest neighbours are to be searched
	 * @param y The y-coordinate of the centre point for which the nearest neighbours are to be searched
	 * @param numClosest The number of neighbours to be searched (i.e. the parameter "k")
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] std::vector<std::pair<const dv::Event *, int32_t>> knnSearch(
		const int32_t x, const int32_t y, const size_t numClosest) const {
		std::vector<std::pair<const dv::Event *, int32_t>> eventsAndDistances;

		const std::array<int32_t, 2> centre = {x, y};

		if (mIndex != nullptr) {
			eventsAndDistances.resize(numClosest);

			const auto numNeighbours = mIndex->knnSearch(centre.data(), numClosest, eventsAndDistances);

			if (numNeighbours < numClosest) {
				eventsAndDistances.resize(numNeighbours);
			}
		}

		return eventsAndDistances;
	}

	/**
	 * Searches for all neighbours surrounding centrePoint that are within a certain radius.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param radius The radius
	 * @param eps The search accuracy
	 * @param sorted True if the neighbours should be sorted with respect to their distance to centrePoint (comes with a
	 * significant performance impact)
	 * @return The number of actually found neighbours
	 */
	template<typename T>
	[[nodiscard]] auto radiusSearch(
		const cv::Point_<T> &centrePoint, const int16_t &radius, float eps = 0.0f, bool sorted = false) const {
		return radiusSearch(centrePoint.x, centrePoint.y, radius, eps, sorted);
	}

	/**
	 * Searches for all neighbours surrounding centrePoint that are within a certain radius.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param radius The radius
	 * @param eps The search accuracy
	 * @param sorted True if the neighbours should be sorted with respect to their distance to centrePoint (comes with a
	 * significant performance impact)
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] auto radiusSearch(
		const dv::Event &centrePoint, const int16_t &radius, float eps = 0.0f, bool sorted = false) const {
		return radiusSearch(centrePoint.x(), centrePoint.y(), radius, eps, sorted);
	}

	/**
	 * Searches for all neighbours surrounding centrePoint that are within a certain radius.
	 *
	 * @param centrePoint The point for which the nearest neighbours are to be searched
	 * @param radius The radius
	 * @param eps The search accuracy
	 * @param sorted True if the neighbours should be sorted with respect to their distance to centrePoint (comes with a
	 * significant performance impact)
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] auto radiusSearch(
		const dv::TimedKeyPoint &centrePoint, const int16_t &radius, float eps = 0.0f, bool sorted = false) const {
		return radiusSearch(static_cast<int32_t>(std::round(centrePoint.pt.x())),
			static_cast<int32_t>(std::round(centrePoint.pt.y())), radius, eps, sorted);
	}

	/**
	 * Searches for all neighbours surrounding centrePoint that are within a certain radius.
	 *
	 * @param x The x-coordinate of the centre point for which the nearest neighbours are to be searched
	 * @param y The y-coordinate of the centre point for which the nearest neighbours are to be searched
	 * @param radius The radius
	 * @param eps The search accuracy
	 * @param sorted True if the neighbours should be sorted with respect to their distance to centrePoint (comes with a
	 * significant performance impact)
	 * @return The number of actually found neighbours
	 */
	[[nodiscard]] std::vector<std::pair<const dv::Event *, int32_t>> radiusSearch(
		const int32_t x, int32_t y, const int16_t &radius, float eps = 0.0f, bool sorted = false) const {
		std::vector<std::pair<const dv::Event *, int32_t>> eventsAndDistances;
		eventsAndDistances.reserve(100000);

		const auto params                   = nanoflann::SearchParams(0, eps, sorted);
		const std::array<int32_t, 2> centre = {x, y};

		if (mIndex != nullptr) {
			mIndex->radiusSearch(centre.data(), radius, eventsAndDistances, params);
		}

		eventsAndDistances.shrink_to_fit();

		return eventsAndDistances;
	}

	/**
	 * Returns an iterator to the begin of the EventStore
	 * @return an iterator to the begin of the EventStore
	 */
	[[nodiscard]] dv::EventStore::iterator begin() const noexcept {
		return mData.begin();
	}

	/**
	 * Returns an iterator to the end of the EventStore
	 * @return  an iterator to the end of the EventStore
	 */
	[[nodiscard]] dv::EventStore::iterator end() const noexcept {
		return mData.end();
	}

	/**
	 * Returns the reference to the this object.
	 * Required by the nanoflann adaptors
	 * @return  the reference to "this"
	 */
	[[nodiscard]] const KDTreeEventStoreAdaptor &derived() const {
		return *this;
	}

	/**
	 * Returns the reference to the this object.
	 * Required by the nanoflann adaptors
	 * @return  the reference to "this"
	 */
	[[nodiscard]] KDTreeEventStoreAdaptor &derived() {
		return *this;
	}

	/**
	 * Returns the point count of the event store.
	 * Required by the nanoflann adaptors
	 * @return  the reference to "this"
	 */
	[[nodiscard]] uint32_t kdtree_get_point_count() const {
		return static_cast<uint32_t>(mData.size());
	}

	/**
	 * Returns the dim'th dimension of an event.
	 * Required by the nanoflann adaptors
	 * @return  the reference to "this"
	 */
	[[nodiscard]] int16_t kdtree_get_pt(const dv::Event *event, const size_t dim) const {
		if (dim == 0) {
			return event->x();
		}
		else {
			return event->y();
		}
	}

	/**
	 * Bounding box computation required by the nanoflann adaptors
	 * As the documentation allows for it not being implemented and we don't need it, it was left empty.
	 * @return  false
	 */
	template<class BBOX>
	bool kdtree_get_bbox(BBOX & /*bb*/) const {
		return false;
	}
};

} // namespace dv::containers::kd_tree

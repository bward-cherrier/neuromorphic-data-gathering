#pragma once

#include "../core/concepts.hpp"
#include "../core/core.hpp"
#include "../data/timed_keypoint_base.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace dv::features {

/**
 * This class implement the Arc* corner detector presented in the following paper:
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/277131/RAL2018-camera-ready.pdf
 *
 * @tparam radius1 radius of the first circle on which the timestamps are checked for corner-ness
 * @tparam radius2 radius of the second circle on which the timestamps are checked for corner-ness
 */

namespace internal {
template<size_t T>
struct CircleCoordinates;
} // namespace internal

template<dv::concepts::TimeSurface<dv::EventStore> TimeSurface = dv::TimeSurface, size_t radius1 = 5,
	size_t radius2 = 6>
class ArcCornerDetector {
public:
	using UniquePtr = std::unique_ptr<ArcCornerDetector>;
	using SharedPtr = std::shared_ptr<ArcCornerDetector>;

	ArcCornerDetector() = delete;

	/**
	 * Constructor
	 *
	 * @tparam TIME_SURFACE_ADDITIONAL_ARGS Types of the additional arguments passed to the time surface constructor
	 * @param resolution camera dimensions
	 * @param range the range within which the timestamps of a corner should be for it to be detected as a corner
	 * @param resetTsAtEachIteration set to true if the time surface should be reset at each iteration
	 * @param timeSurfaceAdditionalArgs arguments passed to the time surface constructor in addition to the resolution
	 */
	template<typename... TIME_SURFACE_ADDITIONAL_ARGS>
	ArcCornerDetector(const cv::Size resolution, const typename TimeSurface::Scalar range,
		const bool resetTsAtEachIteration, TIME_SURFACE_ADDITIONAL_ARGS &&...timeSurfaceAdditionalArgs) :
		mTimeSurfaces{
			TimeSurface(resolution, timeSurfaceAdditionalArgs...),
			TimeSurface(resolution, timeSurfaceAdditionalArgs...)
    },
		mCornerRange{range},
		mResetTsAfterDetection{resetTsAtEachIteration},
		mCircles{CircularTimeSurfaceView{internal::CircleCoordinates<std::min(radius1, radius2)>::coords},
			CircularTimeSurfaceView{internal::CircleCoordinates<std::max(radius1, radius2)>::coords}},
		mArcLimits{{ArcLimits{internal::CircleCoordinates<std::min(radius1, radius2)>::coords.size()},
			ArcLimits{internal::CircleCoordinates<std::max(radius1, radius2)>::coords.size()}}} {
	}

	/**
	 * Runs the detection algorithm.
	 *
	 * A corner is defined by two arcs of different radii containing timestamps which satisfy the following conditions:
	 *
	 * - All timestamps that are on the corner are within a range of mCornerRange.
	 * - No timestamp that is outside of this corner is greater than or equal to the minimum timestamp within the corner
	 * - Length of the arc is within the ranges [ArcLimits::MIN_ARC_SIZE_FACTOR * circumference,
	 * ArcLimits::MAX_ARC_SIZE_FACTOR * circumference]. \see ArcLimits.
	 *
	 * @param events events
	 * @param roi region of interest
	 * @param mask mask containing zeros for all pixels which should be ignored and nonzero for all others
	 * @return a vector containing the detected keypoints. The response is defined as the difference between the minimum
	 * timestamp within the arc and the maximum timestamp outside of the arc.
	 */
	[[nodiscard]] dv::cvector<dv::TimedKeyPoint> detect(
		const dv::EventStore &events, const cv::Rect &roi, const cv::Mat &mask) {
		dv::cvector<dv::TimedKeyPoint> corners;

		if (mResetTsAfterDetection) {
			mTimeSurfaces[0].reset();
			mTimeSurfaces[1].reset();
		}

		for (const auto &event : events) {
			if (roi.contains(cv::Point2i(event.x(), event.y()))
				&& (mask.empty() || mask.at<uint8_t>(event.y(), event.x()) != 0)
				&& cv::Rect(
					radius2, radius2, mTimeSurfaces[0].cols() - 2 * radius2, mTimeSurfaces[0].rows() - 2 * radius2)
					   .contains(cv::Point2i(event.x(), event.y()))) {
				float response = 0.0f;
				bool isCorner  = false;

				for (uint8_t i = 0; i < mCircles.size(); i++) {
					const auto &circle    = mCircles[i];
					const auto &arcLimits = mArcLimits[i];
					// Definition of a corner: An arc of size x where all timestamps that are outside thereof are no
					// greater than the minimum timestamp inside the arc
					//
					// Therefore we find the maximum timestamp first, so we know that an arc must contain this
					// element
					const auto maxTimestampLoc = std::max_element(circle.mCoords.begin(), circle.mCoords.end(),
						[&circle, &event, this](const auto &circleCoord1, const auto &circleCoord2) {
							return circle.getTimestamp(
									   event, circleCoord1, mTimeSurfaces[static_cast<size_t>(event.polarity())])
								 < circle.getTimestamp(
									 event, circleCoord2, mTimeSurfaces[static_cast<size_t>(event.polarity())]);
						});

					const auto maxTimestampValue = circle.getTimestamp(
						event, *maxTimestampLoc, mTimeSurfaces[static_cast<size_t>(event.polarity())]);

					if (maxTimestampValue != 0) {
						const auto [arcSize, arcBegin, arcEnd, minTimestampInArc]
							= expandArc(maxTimestampLoc, maxTimestampValue, event, circle);

						if (arcLimits.satisfied(arcSize)) {
							const auto maxTimestampOutsideArc
								= checkSurroundingTimestamps(arcBegin, arcEnd, minTimestampInArc, event, circle);

							if (minTimestampInArc > maxTimestampOutsideArc) {
								response += static_cast<float>(minTimestampInArc - maxTimestampOutsideArc)
										  / static_cast<float>(mCircles.size());
								isCorner = true;
							}
							else {
								response = 0.0f;
								isCorner = false;
								break;
							}
						}
						else {
							response = 0.0f;
							isCorner = false;
							break;
						}
					}
				}

				if (isCorner) {
					corners.emplace_back(dv::Point2f(event.x(), event.y()), std::max(radius1, radius2), 0, response, 0,
						0, event.timestamp());
				}
			}

			mTimeSurfaces[static_cast<int32_t>(event.polarity())] << event;
		}

		return corners;
	}

	/**
	 * Returns the TimeSurface for a given polarity
	 * @param polarity the polarity
	 * @return the requested time surface
	 */
	[[nodiscard]] auto getTimeSurface(const bool polarity) const {
		return mTimeSurfaces[static_cast<size_t>(polarity)];
	}

private:
	std::array<TimeSurface, 2> mTimeSurfaces;
	int64_t mCornerRange;
	bool mResetTsAfterDetection;

	class CircularTimeSurfaceView {
	public:
		using CoordVector = std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>;

		explicit CircularTimeSurfaceView(CoordVector &coords) : mCoords{coords.begin(), coords.end()} {
		}

		explicit CircularTimeSurfaceView(CoordVector &&coords) : mCoords{coords.begin(), coords.end()} {
		}

		[[nodiscard]] auto getTimestamp(
			const dv::Event &e, const Eigen::Vector2i &circleCoords, const TimeSurface &ts) const {
			return ts(e.y() + circleCoords.y(), e.x() + circleCoords.x());
		}

		template<typename ITERATOR>
		[[nodiscard]] auto circularIncrement(const ITERATOR it) const {
			const auto incremented = it + 1;
			return incremented == mCoords.end() ? mCoords.begin() : incremented;
		}

		template<typename ITERATOR>
		[[nodiscard]] auto circularDecrement(const ITERATOR it) const {
			return it == mCoords.begin() ? mCoords.end() - 1 : it - 1;
		}

		CoordVector mCoords;
	};

	std::array<CircularTimeSurfaceView, 2> mCircles;

	class ArcLimits {
	public:
		explicit ArcLimits(const size_t circumference) :
			mCircumference{circumference},
			mMinSize{static_cast<size_t>(std::round(MIN_ARC_SIZE_FACTOR * static_cast<float>(circumference)))},
			mMaxSize{static_cast<size_t>(std::round(MAX_ARC_SIZE_FACTOR * static_cast<float>(circumference)))} {
		}

		[[nodiscard]] auto satisfied(const size_t arcSize) const {
			const auto invertedArcSize = static_cast<int32_t>(mCircumference) - static_cast<int32_t>(arcSize);

			return (((arcSize >= mMinSize) && (arcSize <= mMaxSize))
					|| ((invertedArcSize >= mMinSize) && ((invertedArcSize <= mMaxSize))));
		}

	private:
		static constexpr float MIN_ARC_SIZE_FACTOR = 0.125f;
		static constexpr float MAX_ARC_SIZE_FACTOR = 0.4f;

		const size_t mCircumference;
		const size_t mMinSize;
		const size_t mMaxSize;
	};

	std::array<ArcLimits, 2> mArcLimits;

	[[nodiscard]] auto insideCorner(const int64_t ts1, const int64_t ts2) {
		return std::abs(ts1 - ts2) < mCornerRange;
	}

	template<typename ITERATOR>
	[[nodiscard]] auto expandArc(const ITERATOR &maxTimestampLoc, const int64_t maxTimestampValue,
		const dv::Event &event, const CircularTimeSurfaceView &circle) {
		// start at the max timestamp, as the arc must contain this element
		auto arcBegin = maxTimestampLoc;
		auto arcEnd   = maxTimestampLoc;

		// As the maximum timestamp is included in the arc by default, the initial arc size is 1
		int16_t arcSize = 1;

		auto minTimestampInArc = maxTimestampValue;

		bool endFound   = false;
		bool beginFound = false;

		do {
			if (!beginFound) {
				const auto beginCandidate = circle.circularDecrement(arcBegin);
				const auto beginCandidateValue
					= circle.getTimestamp(event, *beginCandidate, mTimeSurfaces[static_cast<size_t>(event.polarity())]);

				if (insideCorner(beginCandidateValue, minTimestampInArc)) {
					arcBegin = beginCandidate;
					arcSize++;
					minTimestampInArc
						= std::min(static_cast<int64_t>(beginCandidateValue), static_cast<int64_t>(minTimestampInArc));
				}
				else {
					beginFound = true;
				}
			}

			if (!endFound) {
				const auto endCandidate = circle.circularIncrement(arcEnd);
				const auto endCandidateValue
					= circle.getTimestamp(event, *endCandidate, mTimeSurfaces[static_cast<size_t>(event.polarity())]);

				if (insideCorner(endCandidateValue, minTimestampInArc)) {
					arcEnd = endCandidate;
					arcSize++;
					minTimestampInArc
						= std::min(static_cast<int64_t>(endCandidateValue), static_cast<int64_t>(minTimestampInArc));
				}
				else {
					endFound = true;
				}
			}
		}
		while ((arcSize < circle.mCoords.size()) && !(beginFound && endFound) && (arcBegin != arcEnd));

		return std::make_tuple(arcSize, arcBegin, arcEnd, minTimestampInArc);
	}

	template<typename ITERATOR>
	[[nodiscard]] auto checkSurroundingTimestamps(const ITERATOR &arcBegin, const ITERATOR arcEnd,
		const int64_t minTimestampInArc, const dv::Event &event, const CircularTimeSurfaceView &circle) {
		auto maxTimestampOutsideArc = std::numeric_limits<int64_t>::min();

		for (auto it = circle.circularIncrement(arcEnd); it != arcBegin; it = circle.circularIncrement(it)) {
			const auto timestamp
				= circle.getTimestamp(event, *it, mTimeSurfaces[static_cast<size_t>(event.polarity())]);

			maxTimestampOutsideArc
				= std::max(static_cast<int64_t>(timestamp), static_cast<int64_t>(maxTimestampOutsideArc));

			if (timestamp > minTimestampInArc) {
				break;
			}
		}

		return maxTimestampOutsideArc;
	}
};

static_assert(
	dv::concepts::DVFeatureDetectorAlgorithm<dv::features::ArcCornerDetector<dv::TimeSurface>, dv::EventStore>);

static_assert(dv::concepts::DVFeatureDetectorAlgorithm<dv::features::ArcCornerDetector<dv::SpeedInvariantTimeSurface>,
	dv::EventStore>);

namespace internal {
template<size_t radius>
struct CircleCoordinates {
	static_assert(
		radius == 3 || radius == 4 || radius == 5 || radius == 6 || radius == 7, "This circle radius is not supported");
};

template<>
struct CircleCoordinates<3> {
	inline static std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> coords{
		{Eigen::Vector2i{0, 3}, Eigen::Vector2i{1, 3}, Eigen::Vector2i{2, 2}, Eigen::Vector2i{3, 1},
         Eigen::Vector2i{3, 0}, Eigen::Vector2i{3, -1}, Eigen::Vector2i{2, -2}, Eigen::Vector2i{1, -3},
         Eigen::Vector2i{0, -3}, Eigen::Vector2i{-1, -3}, Eigen::Vector2i{-2, -2}, Eigen::Vector2i{-3, -1},
         Eigen::Vector2i{-3, 0}, Eigen::Vector2i{-3, 1}, Eigen::Vector2i{-2, 2}, Eigen::Vector2i{-1, 3}}
    };
};

template<>
struct CircleCoordinates<4> {
	inline static std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> coords{
		{Eigen::Vector2i{0, 4}, Eigen::Vector2i{1, 4}, Eigen::Vector2i{2, 3}, Eigen::Vector2i{3, 2},
         Eigen::Vector2i{4, 1}, Eigen::Vector2i{4, 0}, Eigen::Vector2i{4, -1}, Eigen::Vector2i{3, -2},
         Eigen::Vector2i{2, -3}, Eigen::Vector2i{1, -4}, Eigen::Vector2i{0, -4}, Eigen::Vector2i{-1, -4},
         Eigen::Vector2i{-2, -3}, Eigen::Vector2i{-3, -2}, Eigen::Vector2i{-4, -1}, Eigen::Vector2i{-4, 0},
         Eigen::Vector2i{-4, 1}, Eigen::Vector2i{-3, 2}, Eigen::Vector2i{-2, 3}, Eigen::Vector2i{-1, 4}}
    };
};

template<>
struct CircleCoordinates<5> {
	inline static std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> coords{
		{Eigen::Vector2i{0, 5}, Eigen::Vector2i{1, 5}, Eigen::Vector2i{2, 5}, Eigen::Vector2i{3, 4},
         Eigen::Vector2i{4, 3}, Eigen::Vector2i{5, 2}, Eigen::Vector2i{5, 1}, Eigen::Vector2i{5, 0},
         Eigen::Vector2i{5, -1}, Eigen::Vector2i{5, -2}, Eigen::Vector2i{4, -3}, Eigen::Vector2i{3, -4},
         Eigen::Vector2i{2, -5}, Eigen::Vector2i{1, -5}, Eigen::Vector2i{0, -5}, Eigen::Vector2i{-1, -5},
         Eigen::Vector2i{-2, -5}, Eigen::Vector2i{-3, -4}, Eigen::Vector2i{-4, -3}, Eigen::Vector2i{-5, -2},
         Eigen::Vector2i{-5, -1}, Eigen::Vector2i{-5, 0}, Eigen::Vector2i{-5, 1}, Eigen::Vector2i{-5, 2},
         Eigen::Vector2i{-4, 3}, Eigen::Vector2i{-3, 4}, Eigen::Vector2i{-2, 5}, Eigen::Vector2i{-1, 5}}
    };
};

template<>
struct CircleCoordinates<6> {
	inline static std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> coords{
		{Eigen::Vector2i{0, 6}, Eigen::Vector2i{1, 6}, Eigen::Vector2i{2, 6}, Eigen::Vector2i{3, 5},
         Eigen::Vector2i{4, 4}, Eigen::Vector2i{5, 3}, Eigen::Vector2i{6, 2}, Eigen::Vector2i{6, 1},
         Eigen::Vector2i{6, 0}, Eigen::Vector2i{6, -1}, Eigen::Vector2i{6, -2}, Eigen::Vector2i{5, -3},
         Eigen::Vector2i{4, -4}, Eigen::Vector2i{3, -5}, Eigen::Vector2i{2, -6}, Eigen::Vector2i{1, -6},
         Eigen::Vector2i{0, -6}, Eigen::Vector2i{-1, -6}, Eigen::Vector2i{-2, -6}, Eigen::Vector2i{-3, -5},
         Eigen::Vector2i{-4, -4}, Eigen::Vector2i{-5, -3}, Eigen::Vector2i{-6, -2}, Eigen::Vector2i{-6, -1},
         Eigen::Vector2i{-6, 0}, Eigen::Vector2i{-6, 1}, Eigen::Vector2i{-6, 2}, Eigen::Vector2i{-5, 3},
         Eigen::Vector2i{-4, 4}, Eigen::Vector2i{-3, 5}, Eigen::Vector2i{-2, 6}, Eigen::Vector2i{-1, 6}}
    };
};

template<>
struct CircleCoordinates<7> {
	inline static std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> coords{
		{Eigen::Vector2i{0, 7}, Eigen::Vector2i{1, 7}, Eigen::Vector2i{2, 7}, Eigen::Vector2i{3, 7},
         Eigen::Vector2i{4, 6}, Eigen::Vector2i{5, 5}, Eigen::Vector2i{6, 4}, Eigen::Vector2i{7, 3},
         Eigen::Vector2i{7, 2}, Eigen::Vector2i{7, 1}, Eigen::Vector2i{7, 0}, Eigen::Vector2i{7, -1},
         Eigen::Vector2i{7, -2}, Eigen::Vector2i{7, -3}, Eigen::Vector2i{6, -4}, Eigen::Vector2i{5, -5},
         Eigen::Vector2i{4, -6}, Eigen::Vector2i{3, -7}, Eigen::Vector2i{2, -7}, Eigen::Vector2i{1, -7},
         Eigen::Vector2i{0, -7}, Eigen::Vector2i{-1, -7}, Eigen::Vector2i{-2, -7}, Eigen::Vector2i{-3, -7},
         Eigen::Vector2i{-4, -6}, Eigen::Vector2i{-5, -5}, Eigen::Vector2i{-6, -4}, Eigen::Vector2i{-7, -3},
         Eigen::Vector2i{-7, -2}, Eigen::Vector2i{-7, -1}, Eigen::Vector2i{-7, 0}, Eigen::Vector2i{-7, 1},
         Eigen::Vector2i{-7, 2}, Eigen::Vector2i{-7, 3}, Eigen::Vector2i{-6, 4}, Eigen::Vector2i{-5, 5},
         Eigen::Vector2i{-4, 6}, Eigen::Vector2i{-3, 7}, Eigen::Vector2i{-2, 7}, Eigen::Vector2i{-1, 7},
         Eigen::Vector2i{0, 7}}
    };
};

} // namespace internal

} // namespace dv::features

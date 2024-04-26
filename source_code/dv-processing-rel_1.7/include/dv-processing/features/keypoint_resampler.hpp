#pragma once

#include "../core/concepts.hpp"
#include "../data/boost_geometry_interop.hpp"

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace dv::features {

/**
 * Create a feature resampler, which resamples given keypoints with
 * homogenous distribution in pixel space.
 *
 * Implementation was inspired by:
 * https://github.com/BAILOOL/ANMS-Codes
 */
class KeyPointResampler {
protected:
	// Maintain previous final width
	float mPreviousSolution = -1.f;

	float mRows;
	float mCols;
	float mTolerance = 0.1f;

	typedef std::pair<dv::Point2f, size_t> RangeValue;

public:
	/**
	 * Initialize resampler with given resolution.
	 * @param resolution		Image resolution
	 */
	explicit KeyPointResampler(const cv::Size &resolution) :
		mRows(static_cast<float>(resolution.height)),
		mCols(static_cast<float>(resolution.width)) {
	}

	/**
	 * Perform resampling on given keypoints.
	 * @param keyPoints 		Prior keypoints.
	 * @param numRetPoints 		Number of expected keypoints, the exact number
	 * 							of output keypoints can vary to configured tolerance
	 * 							value (@sa setTolerance)
	 * @return 					Resampled keypoints
	 */
	template<class KeyPointVectorType>
	requires dv::concepts::KeyPointVector<KeyPointVectorType>
		  or dv::concepts::Coordinate2DMutableIterable<KeyPointVectorType>
	inline KeyPointVectorType resample(const KeyPointVectorType &keyPoints, size_t numRetPoints) {
		if (numRetPoints == 0 || keyPoints.empty()) {
			return KeyPointVectorType();
		}

		if (numRetPoints >= keyPoints.size()) {
			return keyPoints;
		}

		namespace bg  = boost::geometry;
		namespace bgi = boost::geometry::index;

		// Typecasting
		auto K = static_cast<float>(numRetPoints);

		// several temp expression variables to simplify solution equation
		float exp1 = mRows + mCols + 2.f * K;
		float exp2 = (4.f * mCols + 4.f * K + 4.f * mRows * K + mRows * mRows + mCols * mCols - 2.f * mRows * mCols
					  + 4.f * mRows * mCols * K);
		float exp3 = std::sqrt(exp2);
		float exp4 = K - 1.f;

		float sol1 = -std::round((exp1 + exp3) / exp4);
		float sol2 = -std::round((exp1 - exp3) / exp4);

		// binary search range initialization with positive solution
		float high = std::max(sol1, sol2);
		float low  = std::floor(std::sqrt(static_cast<float>(keyPoints.size()) / K));

		// Packed rangetree search algorithm
		std::vector<RangeValue> values;
		for (size_t i = 0; i < keyPoints.size(); i++) {
			if constexpr (dv::concepts::KeyPointVector<KeyPointVectorType>) {
				if constexpr (dv::concepts::Coordinate2DAccessors<decltype(keyPoints[i].pt)>) {
					values.emplace_back(dv::Point2f(keyPoints[i].pt.x(), keyPoints[i].pt.y()), i);
				}
				else {
					values.emplace_back(dv::Point2f(keyPoints[i].pt.x, keyPoints[i].pt.y), i);
				}
			}
			else {
				if constexpr (dv::concepts::Coordinate2DAccessors<decltype(keyPoints[i])>) {
					values.emplace_back(dv::Point2f(keyPoints[i].x(), keyPoints[i].y()), i);
				}
				else {
					values.emplace_back(dv::Point2f(keyPoints[i].x, keyPoints[i].y), i);
				}
			}
		}
		bgi::rtree<RangeValue, bgi::rstar<4>> rangetree(values.begin(), values.end());

		bool complete   = false;
		auto Kmin       = static_cast<size_t>(std::round(K - (K * mTolerance)));
		auto Kmax       = static_cast<size_t>(std::round(K + (K * mTolerance)));
		float width     = 0.f;
		float prevwidth = -1.f;

		std::vector<size_t> result;
		result.reserve(keyPoints.size());
		dv::BoundingBox queryBox(0, 0.f, 0.f, 0.f, 0.f, 0.f, {});

		while (!complete) {
			std::vector<bool> included(values.size(), true);
			bool usePreviousSolution = (prevwidth < 0.f && mPreviousSolution > 0.f);

			width = (usePreviousSolution ? mPreviousSolution : (low + ((high - low) / 2.f)));
			// needed to reassure the same width is not repeated again
			if (width == prevwidth || low > high) {
				break;
			}
			result.clear();

			for (size_t i = 0; i < values.size(); ++i) {
				if (included[i]) {
					included[i] = false;
					result.push_back(i);
					const auto &pt = values[i].first;
					// defining square boundaries around the point
					queryBox.topLeftX     = pt.x() - width;
					queryBox.bottomRightX = pt.x() + width;
					queryBox.topLeftY     = pt.y() - width;
					queryBox.bottomRightY = pt.y() + width;

					std::vector<RangeValue> result_s;
					rangetree.query(bgi::covered_by(queryBox), std::back_inserter(result_s));

					for (const auto &j : result_s) {
						if (included[j.second]) {
							included[j.second] = false;
						}
					}
				}
			}
			if (result.size() >= Kmin && result.size() <= Kmax) { // solution found
				complete = true;
			}
			else if (!usePreviousSolution) {
				// Update the search range only if we have executed the previous solution search
				if (result.size() < Kmin) {
					high = static_cast<int16_t>(width - 1); // update binary search range
				}
				else {
					low = static_cast<int16_t>(width + 1);
				}
			}
			prevwidth = width;
		}

		// Buffer the previous solution as a prior for the next search
		mPreviousSolution = width;

		// retrieve final keypoints
		KeyPointVectorType output;
		output.reserve(result.size());
		for (size_t i : result) {
			output.push_back(keyPoints[i]);
		}

		return output;
	}

	/**
	 * Get currently set tolerance for output keypoint count.
	 * @return		Tolerance value
	 */
	[[nodiscard]] float getTolerance() const {
		return mTolerance;
	}

	/**
	 * Set a new output size tolerance value.
	 *
	 * The algorithm search for an optimal distance between keypoints so the resulting
	 * vector would contain the expected amount of keypoints. This search is performed
	 * with a given tolerance, by default - 0.1 (so by default the final resampled amount
	 * of events will be within +/-10% of requested amount).
	 *
	 * @param tolerance 	Output keypoint amount tolerance value.
	 */
	void setTolerance(const float tolerance) {
		mTolerance = tolerance;
	}
};

} // namespace dv::features

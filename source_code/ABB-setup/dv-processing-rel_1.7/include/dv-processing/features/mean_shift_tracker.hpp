#pragma once

#include "../core/core.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "feature_detector.hpp"
#include "redetection_strategy.hpp"
#include "tracker_base.hpp"

namespace dv::features {

/**
 * Track event blobs using mean shift algorithm on time surface event data.
 */
class MeanShiftTracker : public TrackerBase {
public:
	/**
	 * Constructor for mean shift tracker using Epanechnikov kernel as weights for the time surface of events used to
	 * update track location.
	 * The kernel weights have highest value on the previous track location.
	 * This assumption is based on the idea that the new track location is "close" to last track location.
	 * The consecutive track updates are performed until the maximum number of iteration is reached or the shift between
	 * consecutive updates is below a threshold.
	 *
	 * @param resolution full image plane resolution
	 * @param bandwidth search window dimension size. The search area is a square. The square side is 2 * bandwidth
	 * and center the current track location
	 * @param timeWindow look back time from latest event: used to generate normalized time surface. All events older
	 * than (latestEventTime-timeWindow) will be discarded
	 * @param redetectionStrategy strategy used to decide if and when to re-detect interesting points to track
	 * @param detector detector used to re-detect tracks if redetection strategy uis defined and should happen
	 * @param stepSize weight applied to shift to compute new track location. This value is in range (0, 1). A value of
	 * 0 means that no shift is performed. A value of 1 means that the new candidate center is directly assigned as new
	 * center
	 * @param weightMultiplier scaling factor for Epanechnikov weights used in the computation of the mean shift cost
	 * update
	 * @param convergenceNorm shift value below which search will not continue (this value is named "mode" in the docs)
	 * @param maxIters maximum number of search iterations for one track update
	 */
	MeanShiftTracker(const cv::Size &resolution, const int bandwidth, const dv::Duration timeWindow,
		RedetectionStrategy::UniquePtr redetectionStrategy = nullptr,
		std::unique_ptr<EventFeatureBlobDetector> detector = nullptr, const float stepSize = 0.5f,
		const float weightMultiplier = 1.f, float convergenceNorm = 0.01f, int maxIters = 2000) :
		mBandwidth(bandwidth),
		mSurface(resolution),
		mTimeWindow(timeWindow),
		mResolution(resolution),
		mWeightMultiplier(weightMultiplier),
		mConvergenceNorm(convergenceNorm),
		mMaxIters(maxIters) {
		setStepSize(stepSize);
		setDetector(std::move(detector));
		setRedetectionStrategy(std::move(redetectionStrategy));
	}

	/**
	 * Add events to time surface and update last batch of events fed to the tracker.
	 * @param store new incoming events for the tracker.
	 */
	void accept(const dv::EventStore &store) {
		mSurface.accept(store);
		mEvents = store;
	}

	/**
	 * Compute new centers based on area with highest event density. The density is weighted by the event timestamp:
	 * newer timestamps have higher weight.
	 * @return structure containing new track locations as a vector of dv::TimedKeyPoint
	 */
	[[nodiscard]] Result::SharedPtr track() override {
		const cv::Mat normalizedTimeSurface = mSurface.getOCVMatScaled(mTimeWindow.count());

		if ((!lastFrameResults) || (lastFrameResults->keypoints.empty())) {
			auto newTracks = mDetector->runDetection(mEvents, getMaxTracks());
			for (auto &track : newTracks) {
				track.class_id   = mLastFreeClassId;
				mLastFreeClassId += 1;
			}

			return std::make_shared<TrackerBase::Result>(mEvents.getLowestTime(), newTracks, true);
		}

		// shift tracks to new position - ID handling is done inside the update
		auto result = updateTracks(normalizedTimeSurface);

		// detect new tracks if expected
		if (mRedetectionStrategy && mRedetectionStrategy->decideRedetection(*this)) {
			// update current set of tracks stored in result variable
			runRedetection(result);
		}

		return result;
	}

	/**
	 * Define redetection strategy used to re-detect interesting points to track.
	 * @param redetectionStrategy type of redetection to use (check redetection_strategy.hpp for available types of
	 * re-detections)
	 */
	void setRedetectionStrategy(RedetectionStrategy::UniquePtr redetectionStrategy) {
		mRedetectionStrategy = std::move(redetectionStrategy);
	}

	/**
	 * Define detector used to detect interesting points to track (if redetection should happen)
	 * @param detector detector for new interesting points to track
	 */
	void setDetector(std::unique_ptr<EventFeatureBlobDetector> detector) {
		if (detector) {
			mDetector = std::move(detector);
		}
		else {
			const auto newDetector = std::make_shared<dv::features::EventBlobDetector>(mResolution);
			mDetector              = std::make_unique<dv::features::EventFeatureBlobDetector>(mResolution, newDetector);
		}
	}

	/**
	 * Getter for bandwidth value that defines the search area for a new track.
	 * For detailed information on how the area is computed please check related parameter in constructor.
	 * @return search window dimension size.
	 */
	[[nodiscard]] int getBandwidth() const {
		return mBandwidth;
	}

	/**
	 * Setter for bandwidth value.
	 * @param bandwidth search window dimension size.
	 */
	void setBandwidth(const int bandwidth) {
		if (bandwidth < 0) {
			throw dv::exceptions::InvalidArgument<int>(
				"Trying to set negative bandwidth value which results in unknown algorithm results.", bandwidth);
		}
		mBandwidth = bandwidth;
	}

	/**
	 * Get time window duration used to normalize time surface.
	 * @return value of time window use to generate normalized time surface
	 */
	[[nodiscard]] dv::Duration getTimeWindow() const {
		return mTimeWindow;
	}

	/**
	 * Setter for time window duration for time surface normalization.
	 * @param timeWindow size of window
	 */
	void setTimeWindow(const dv::Duration timeWindow) {
		mTimeWindow = timeWindow;
	}

	/**
	 * Get multiplier value used for track location update. Given a computed shift to be applied to a track, the actual
	 * shift performed is given by mStepSize * shift.
	 * @return scaling value applied to the spatial interval computed between current and new track position at
	 * consecutive updates
	 */
	[[nodiscard]] float getStepSize() const {
		return mStepSize;
	}

	/**
	 * Setter for learning rate for motion towards new center during one mean shift iteration.
	 * Please check the same parameter in the constructor description for detailed information.
	 * @param stepSize weight applied to shift to compute new track location.
	 */
	void setStepSize(const float stepSize) {
		if ((stepSize < 0.f) || (stepSize > 1.f)) {
			throw dv::exceptions::InvalidArgument<float>(
				"Setting a step size out of the valid [0.0; 1.0] range.", stepSize);
		}

		mStepSize = stepSize;
	}

	/**
	 * Getter for weight multiplier used to adjust weight of each time surface value in the mean shift update. If
	 * multiplier is smaller than 1, the cost values for each location are shrink, whereas if the multiplier is larger
	 * than 1, the difference between time surface intensities will be larger.
	 * @return weight multiplier value
	 */
	[[nodiscard]] float getWeightMultiplier() const {
		return mWeightMultiplier;
	}

	/**
	 * Setter for scaling factor used in the computation of the mean shift cost update.
	 * @param multiplier scaling factor value
	 */
	void setWeightMultiplier(const float multiplier) {
		mWeightMultiplier = multiplier;
	}

	/**
	 * Get norm of distance between consecutive tracks updates. If the distance is smaller than this norm, the track
	 * update is considered to be converged.
	 * @return value of distance norm between consecutive updates
	 */
	[[nodiscard]] float getConvergenceNorm() const {
		return mConvergenceNorm;
	}

	/**
	 * Setter for threshold norm (i.e. mode) between consecutive track updates below which iterations are stopped.
	 * @param norm threshold value
	 */
	void setConvergenceNorm(const float norm) {
		mConvergenceNorm = norm;
	}

	/**
	 * Get maximum number of times track update can be run.
	 * @return value of maximum number of operations for track update
	 */
	[[nodiscard]] int getMaxIterations() const {
		return mMaxIters;
	}

	/**
	 * Setter for maximum number of track updates.
	 * @param maxIters value of maximum number of operations for track update
	 */
	void setMaxIterations(const int maxIters) {
		mMaxIters = maxIters;
	}

private:
	/**
	 * Compute new location for all tracks. If a new position fall inside the area of a new position computed for a
	 * previous track, the track will not be updated. Previous track with its timestamp will be kept.
	 * @param normalizedTimeSurface image representation of event timestamps based on time surface
	 * @return updated track positions
	 */
	[[nodiscard]] Result::SharedPtr updateTracks(const cv::Mat &normalizedTimeSurface) {
		dv::cvector<dv::TimedKeyPoint> newTracks;
		std::vector<dv::Point2f> newCenters;

		for (const auto &inputTrack : lastFrameResults->keypoints) {
			const auto newCandidateCenter = computeShift(inputTrack.pt, normalizedTimeSurface, inputTrack.size);

			const auto isWithinRangeOfNewTracksAlreadyAdded = [&newCandidateCenter, this](const auto newStoredCenters) {
				const float x        = newCandidateCenter->x() - newStoredCenters.x();
				const float y        = newCandidateCenter->y() - newStoredCenters.y();
				const float distance = sqrt(x * x + y * y);
				return distance < static_cast<float>(mBandwidth);
			};

			if (newCandidateCenter.has_value()
				&& std::none_of(newCenters.begin(), newCenters.end(), isWithinRangeOfNewTracksAlreadyAdded)) {
				auto timestamp = mSurface.at(cv::saturate_cast<int16_t>(newCandidateCenter->y()),
					cv::saturate_cast<int16_t>(newCandidateCenter->x()));

				// note that track update happened, here we account for possible imperfections in the time surface of
				// events, due to noise in the event data itself. Specifically, if we update a track but no event
				// happened at the updated track location, it most likely happens due to noisy event data.
				if (timestamp < mEvents.getLowestTime()) {
					timestamp = mEvents.getLowestTime();
				}

				const auto newTrack = dv::TimedKeyPoint(*newCandidateCenter, inputTrack.size, inputTrack.angle,
					inputTrack.response, inputTrack.octave, inputTrack.class_id, timestamp);

				newCenters.push_back(*newCandidateCenter);
				newTracks.push_back(newTrack);
			}
			else {
				newTracks.push_back(inputTrack);
			}
		}

		return std::make_shared<TrackerBase::Result>(mEvents.getLowestTime(), newTracks, false);
	}

	/**
	 * Compute new track location.
	 * Note: kernel weights are updated only if the search window changed size or if it intersects the boundaries of the
	 * image plane. This decision has been made for performance reasons and should not affect the final result as long
	 * as the new track position is "close enough", to the starting position.
	 * @param center previous track location
	 * @param timeSurface Matrix containing normalized time surface values
	 * @param trackSize dimension of track determining kernel size
	 * @return new final track location if value is valid, std::nullopt is returned if the search area has no event data
	 * inside it.
	 */
	[[nodiscard]] std::optional<dv::Point2f> computeShift(
		const dv::Point2f &center, const cv::Mat &timeSurface, const float trackSize) {
		cv::Point2f shift;
		dv::Point2f newCenterLastIter = center;

		cv::Mat kernelWeights;
		int32_t numIter = 0;
		do {
			auto croppedArea        = findSpatialWindow(newCenterLastIter, timeSurface);
			const auto &roi         = croppedArea.second;
			const bool isAtBoundary = (roi.x == 0) || (roi.y == 0) || (roi.x + 2 * mBandwidth > mResolution.width)
								   || (roi.y + 2 * mBandwidth > mResolution.height);
			if ((roi.size() != kernelWeights.size()) || isAtBoundary) {
				kernelWeights = kernelEpanechnikovWeights(newCenterLastIter, roi, trackSize);
			}

			auto modeLocal = updateCenterLocation(croppedArea.first, kernelWeights);
			if (!modeLocal.has_value()) {
				return std::nullopt;
			}
			// find location in full image plane
			const dv::Point2f mode(
				modeLocal->x() + static_cast<float>(roi.x), modeLocal->y() + static_cast<float>(roi.y));

			shift.x           = (mode.x() - newCenterLastIter.x()) * mStepSize;
			shift.y           = (mode.y() - newCenterLastIter.y()) * mStepSize;
			newCenterLastIter = dv::Point2f(newCenterLastIter.x() + shift.x, newCenterLastIter.y() + shift.y);

			numIter++;
		}
		while ((static_cast<float>(norm(shift)) > mConvergenceNorm) && (numIter < mMaxIters));

		return newCenterLastIter;
	}

	/**
	 * Compute mode (i.e. track location).
	 * @param spatialWindow image plane sub-matrix in which the center will be updated
	 * @param kernelWeights weights of Epanechnikov kernel applied to each time surface location inside the given
	 * spatial window
	 * @return new track location
	 */
	[[nodiscard]] std::optional<dv::Point2f> updateCenterLocation(
		const cv::Mat &spatialWindow, const cv::Mat &kernelWeights) const {
		cv::Point2f mean(0.f, 0.f);
		float weightTot = 0.f;

		// mode : weighted average of non-zero coords
		for (int y = 0; y < spatialWindow.rows; y++) {
			for (int x = 0; x < spatialWindow.cols; x++) {
				const auto weight = static_cast<float>(spatialWindow.at<uint8_t>(y, x)) * kernelWeights.at<float>(y, x);
				weightTot         += weight;
				mean.x            += weight * static_cast<float>(x);
				mean.y            += weight * static_cast<float>(y);
			}
		}

		if (weightTot == 0) {
			return std::nullopt;
		}

		return dv::Point2f(mean.x / weightTot, mean.y / weightTot);
	}

	/**
	 * Compute Epanechnikov kernel with highest peak at center location.
	 * @param center location of the observed sample, i.e. location where kernel will have highest response
	 * @param window window in which kernel weights will be computed
	 * @param cutOffValue distance from center after which all kernel weights will be zero. For more information about
	 * the kernel please check equation 3.54 from
	 * http://sfb649.wiwi.hu-berlin.de/fedc_homepage/xplore/ebooks/html/spm/spmhtmlnode18.html
	 * @return matrix with weights of Epanechnikov kernel
	 */
	[[nodiscard]] cv::Mat kernelEpanechnikovWeights(
		const dv::Point2f &center, const cv::Rect &window, const float cutOffValue) const {
		cv::Mat weightMat(window.size(), CV_32F, 1);
		if (cutOffValue == 0) {
			return weightMat;
		}

		const auto xCenter = static_cast<int>(center.x()) - window.x;
		const auto yCenter = static_cast<int>(center.y()) - window.y;

		for (int y = 0; y < window.height; y++) {
			for (int x = 0; x < window.width; x++) {
				const auto xValue   = static_cast<float>(xCenter - x);
				const auto yValue   = static_cast<float>(yCenter - y);
				const bool xInRange = (abs(xValue) / cutOffValue) <= 1;
				const bool yInRange = (abs(yValue) / cutOffValue) <= 1;

				if (xInRange && yInRange) {
					const float num = 9 * (1 - (xValue / cutOffValue) * (xValue / cutOffValue))
									* (1 - (yValue / cutOffValue) * (yValue / cutOffValue));
					const float den           = 16 * cutOffValue * cutOffValue;
					weightMat.at<float>(y, x) = num / den * mWeightMultiplier;
				}
				else {
					weightMat.at<int>(y, x) = 0;
				}
			}
		}

		return weightMat;
	}

	/**
	 * Compute area in which the new track position will be searched. This area depends on the bandwidth value.
	 * The search area is defined as the square around the center value with size of one side as 2*bandwidth.
	 * We return the selected area as first argument and the roi in the full image plane to be able to retrieve
	 * coordinates of selected area in the original image space.
	 * @param center previous track center around which we define the search area
	 * @param image full image plane data
	 * @return pair containing as first output the matrix block containing the data inside the image defined by the
	 * rectangle returned as second output
	 */
	[[nodiscard]] std::pair<cv::Mat, cv::Rect> findSpatialWindow(
		const dv::Point2f &center, const cv::Mat &image) const {
		const cv::Rect fullArea(0, 0, image.cols, image.rows);
		const cv::Rect areaOfInterest(static_cast<int>(center.x()) - mBandwidth,
			static_cast<int>(center.y()) - mBandwidth, mBandwidth * 2, mBandwidth * 2);
		const cv::Rect roi   = fullArea & areaOfInterest;
		const cv::Mat roiMat = image(roi);

		return std::make_pair(roiMat, roi);
	}

	/**
	 * Re-detect interesting points
	 * @param result current set of tracks to which new detections will be added
	 */
	void runRedetection(Result::SharedPtr &result) {
		auto detectedTracks = mDetector->runDetection(mEvents, getMaxTracks());
		if (!result) {
			return;
		}
		result->asKeyFrame = true;
		// add detected tracks if not already existing and assign new class id
		for (auto &newCandidate : detectedTracks) {
			auto isWithinRangeOfOldTracks = [&newCandidate, this](const auto previousCenters) {
				double distance = sqrt(pow(newCandidate.pt.x() - previousCenters.pt.x(), 2)
									   + pow(newCandidate.pt.y() - previousCenters.pt.y(), 2));
				return distance < mBandwidth;
			};

			if (std::none_of(result->keypoints.begin(), result->keypoints.end(), isWithinRangeOfOldTracks)) {
				newCandidate.class_id = mLastFreeClassId;
				mLastFreeClassId      += 1;
				result->keypoints.push_back(newCandidate);
			}
		}
	}

	/// parameter defining search window size for each track update
	int mBandwidth;

	/// event time surface
	dv::TimeSurface mSurface;

	/// time window of events to generate the normalized time surface from
	dv::Duration mTimeWindow;

	float mStepSize;

	cv::Size mResolution;

	/// latest batch of events fed to the tracker
	dv::EventStore mEvents = dv::EventStore();

	/// detector used if no track has been detected or redetection is expected to happen
	std::unique_ptr<EventFeatureBlobDetector> mDetector;

	/// value used to keep track of first free ID for a new track
	int32_t mLastFreeClassId = 0;

	/// type of redetection strategy used to detect new interesting points to track
	RedetectionStrategy::UniquePtr mRedetectionStrategy = nullptr;

	/// Weight multiplier used to adjust weight of each point in the mean shift update. If multiplier is smaller than 1,
	/// the cost values for each location are shrink, whereas if the multiplier is larger than 1, the difference
	/// between points with lower intensity in the time surface will be increased from ones with larger intensity values
	float mWeightMultiplier;

	/// shift value below which search will not continue
	float mConvergenceNorm;

	/// maximum number of search iterations for one track update
	int mMaxIters;
};

static_assert(concepts::Accepts<MeanShiftTracker, dv::EventStore>);
} // namespace dv::features

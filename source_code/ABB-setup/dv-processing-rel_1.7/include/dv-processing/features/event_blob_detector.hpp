#pragma once

#include "../core/event.hpp"
#include "../core/frame/accumulator.hpp"
#include "../data/utilities.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <utility>

namespace dv::features {

/**
 * Event-based blob detector performing detection on accumulated event images.
 */
class EventBlobDetector {
public:
	/**
	 * Create a reasonable default blob detector.
	 *
	 * The method creates an instance of cv::SimpleBlobDetector with following parameter values:
	 * - filterByArea     	 = true
	 * - minArea 			 = 10 : minimum area of blobs to be detected - reasonable value to safely detect blobs and
	 * not noise in the accumulated image
	 * - maxArea 			 = 10000
	 * - filterByCircularity = false
	 * - filterByConvexity   = false
	 * - filterByInertia     = false
	 * @return blob detector used by default to detect interesting blobs
	 */
	[[nodiscard]] static cv::Ptr<cv::SimpleBlobDetector> defaultBlobDetector() {
		cv::SimpleBlobDetector::Params blobParams;
		blobParams.filterByArea        = true;
		blobParams.minArea             = 10;
		blobParams.maxArea             = 10000;
		blobParams.filterByCircularity = false;
		blobParams.filterByConvexity   = false;
		blobParams.filterByInertia     = false;

		return cv::SimpleBlobDetector::create(blobParams);
	}

	/**
	 * Constructor for blob detector.
	 *
	 * The detection steps are as following:
	 * 1) Compute accumulated image from events
	 * 2) Apply ROI to the accumulated event image
	 * 3) Down sample image (if pyramidLevel >= 1)
	 * 4) Apply preprocess function (if exists)
	 * 5) Detect blobs
	 * 6) Rescale blobs to original resolution (if pyramidLevel >= 1)
	 * 7) If ROI has an offset from (0,0) of initial image plane, add offset back to bring blobs location in the
	 *    original image space coordinate system
	 * 8) Remove blobs where mask value is 0.
	 *
	 * @param resolution original image plane resolution
	 * @param pyramidLevel integer defining number of down samples applied to the accumulated image.
	 * 					   E.g. if pyramidLevel = 3 --> we down sample the image by a factor of 2 for N=3 times.
	 * 				            this means that an image of size (100, 100) is down sampled to (25, 25) before
	 * performing the blob detection. Note that blob location is always returned in the original resolution size.
	 *
	 * @param preprocess function to be applied to the accumulated image before performing the detection step. The
	 * function modifies the input image passed as argument to the function in place. Internally, the api check that
	 * resolution and type of the image are kept.
	 * @param blobDetector blob detector instance performing the detection step
	 */
	explicit EventBlobDetector(const cv::Size &resolution, const int pyramidLevel = 0,
		std::function<void(cv::Mat &)> preprocess    = {},
		cv::Ptr<cv::SimpleBlobDetector> blobDetector = defaultBlobDetector()) :
		mBlobDetector(std::move(blobDetector)),
		mPyramidLevel(pyramidLevel),
		mPreprocessFcn(std::move(preprocess)),
		mAccumulator(resolution) {
	}

	/**
	 * Detection step.
	 *
	 * @param events data used to create the accumulated image over which blob detection will be applied
	 * @param roi region in which blobs will be searched
	 * @param mask disable any blob detections on coordinates with zero pixel value on the mask.
	 * @return blobs found from blob detector
	 */
	[[nodiscard]] dv::cvector<dv::TimedKeyPoint> detect(
		const dv::EventStore &events, const cv::Rect &roi = cv::Rect(), const cv::Mat &mask = cv::Mat()) {
		if (events.isEmpty()) {
			return {};
		}

		mAccumulator.accept(events);

		// create event image
		const auto frame       = mAccumulator.generateFrame();
		cv::Mat imageToProcess = frame.image;

		if (roi.area() > 0) {
			imageToProcess = imageToProcess(roi);
		}

		// pyrDown
		for (int index = 0; index < mPyramidLevel; index++) {
			cv::pyrDown(imageToProcess, imageToProcess);
		}

		// apply preprocessing if present
		if (mPreprocessFcn) {
			const auto size = imageToProcess.size;
			const auto type = imageToProcess.type();
			mPreprocessFcn(imageToProcess);
			dv::runtime_assert(
				imageToProcess.size == size, "Image size has been modified, this will cause unexpected behaviour.");
			dv::runtime_assert(
				imageToProcess.type() == type, "Image type has been modified, this will cause unexpected behaviour.");
		}

		std::vector<cv::KeyPoint> blobs;
		mBlobDetector->detect(255 - imageToProcess, blobs);

		const float pyramidMultiplier = powf(2.f, static_cast<float>(mPyramidLevel));
		for (auto &kp : blobs) {
			kp.pt   = kp.pt * pyramidMultiplier;
			kp.size = kp.size * pyramidMultiplier;
		}

		if (roi.area() > 0) {
			for (auto &kp : blobs) {
				kp.pt.x += static_cast<float>(roi.x);
				kp.pt.y += static_cast<float>(roi.y);
			}
		}

		if (!mask.empty()) {
			auto iter = blobs.begin();
			while (iter != blobs.end()) {
				if (mask.at<uint8_t>(static_cast<int>(iter->pt.y), static_cast<int>(iter->pt.x)) == 0) {
					iter = blobs.erase(iter);
				}
				else {
					iter++;
				}
			}
		}

		return dv::data::fromCvKeypoints(blobs, frame.timestamp);
	}

private:
	/**
	 * Blob detector instance performing the detection step
	 */
	cv::Ptr<cv::SimpleBlobDetector> mBlobDetector;

	/**
	 * Number of pyrDown applied to the accumulated image
	 */
	int32_t mPyramidLevel;

	/**
	 * Preprocessing function to be applied before the detection step
	 */
	std::function<void(cv::Mat &)> mPreprocessFcn;

	/**
	 * Accumulator generating the image used for blob detection
	 */
	dv::EdgeMapAccumulator mAccumulator;
};

static_assert(dv::concepts::DVFeatureDetectorAlgorithm<dv::features::EventBlobDetector, dv::EventStore>);
static_assert(dv::concepts::FeatureDetectorAlgorithm<dv::features::EventBlobDetector, dv::EventStore>);

} // namespace dv::features

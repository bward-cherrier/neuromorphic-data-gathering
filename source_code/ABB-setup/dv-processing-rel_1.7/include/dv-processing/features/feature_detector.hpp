#pragma once

#include "../core/concepts.hpp"
#include "../core/core.hpp"
#include "../data/timed_keypoint_base.hpp"
#include "../data/utilities.hpp"
#include "event_blob_detector.hpp"
#include "image_pyramid.hpp"
#include "keypoint_resampler.hpp"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace dv::features {
template<class InputType, dv::concepts::FeatureDetectorAlgorithm<InputType> Algorithm>
class FeatureDetector;

using ImagePyrFeatureDetector  = FeatureDetector<dv::features::ImagePyramid, cv::Feature2D>;
using ImageFeatureDetector     = FeatureDetector<dv::Frame, cv::Feature2D>;
using EventFeatureBlobDetector = FeatureDetector<dv::EventStore, EventBlobDetector>;

/**
 * A base class to implement feature detectors on different input types, specifically either images, time surfaces,
 * or event stores. The implementing class should override the `detect` function and output a vector of unordered
 * features with a quality score. The API will handle margin calculations and post processing of the features.
 * @tparam InputType        The type of input that is needed for the detector.
 * @tparam Algorithm 		The underlying detection algorithm, can be an OpenCV::Feature2D algorithm or a custom
 * implementation, as long as it satisfies \see dv::concepts::FeatureDetectorAlgorithm
 */
template<class InputType, dv::concepts::FeatureDetectorAlgorithm<InputType> Algorithm>
class FeatureDetector {
public:
	using ThisType  = FeatureDetector<InputType, Algorithm>;
	using SharedPtr = std::shared_ptr<ThisType>;
	using UniquePtr = std::unique_ptr<ThisType>;

	// As we want to support OpenCV detection algorithms and their factory functions, we need a way to internally
	// distinguish between std::shared_Ptr and cv::Ptr. Using this alias we can do this at compile time and transparent
	// to the user.
	using AlgorithmPtr = typename std::conditional_t<std::is_base_of_v<cv::Feature2D, Algorithm>, cv::Ptr<Algorithm>,
		std::shared_ptr<Algorithm>>;

public:
	/**
	 * Feature post processing step performed after the the features were detected.
	 * Currently available types of post processing:
	 * - None:          Do not perform any postprocessing, all keypoints from detection will be returned.
	 * - TopN:          Retrieve the top number of highest scoring features.
	 * - AdaptiveNMS:   Apply the AdaptiveNMS algorithm to retrieve equally spaced keypoints in
	 *                  pixel space dimensions. More information on the AdaptiveNMS here:
	 *                  original code: https://github.com/BAILOOL/ANMS-Codes
	 *                  paper:
	 * https://www.researchgate.net/publication/323388062_Efficient_adaptive_non-maximal_suppression_algorithms_for_homogeneous_spatial_keypoint_distribution
	 */
	enum class FeaturePostProcessing {
		None,
		TopN,
		AdaptiveNMS
	};

public:
	/**
	 * Create a feature detector.
	 * @param _imageDimensions      Image dimensions.
	 * @param _postProcessing       Post processing step - subsampling of events,
	 *                              @sa FeatureDetectorBase::FeaturePostProcessing
	 * @param _margin               Margin coefficient, it will be multiplied by the width and height
	 *                              of the image to calculate an adaptive border alongside the edges of
	 *                              image, where features should not be detected.
	 */
	FeatureDetector(const cv::Size &_imageDimensions, const AlgorithmPtr &_detector,
		FeaturePostProcessing _postProcessing, float _margin = 0.02f) :
		postProcessing(_postProcessing),
		imageDimensions(_imageDimensions),
		detector(_detector),
		resampler(_imageDimensions) {
		setMargin(_margin);
	}

	/**
	 * Create a feature detector. This constructor defaults post-processing step to AdaptiveNMS
	 * and margin coefficient value of 0.02.
	 * @param _imageDimensions      Image dimensions.
	 */
	explicit FeatureDetector(const cv::Size &_imageDimensions, const AlgorithmPtr &_detector) :
		postProcessing(FeaturePostProcessing::AdaptiveNMS),
		detector(_detector),
		imageDimensions(_imageDimensions),
		resampler(_imageDimensions) {
		setMargin(0.02f);
	}

	/**
	 * Destructor
	 */
	virtual ~FeatureDetector() = default;

	/**
	 * Public detection call. Calls the overloaded `detect` function, applies margin and post processing.
	 * @param input         The input to the detector
	 * @param numPoints     Number of keypoints to be detected
	 * @param mask          Detection mask, detection will be performed where mask value is non-zero.
	 * @return              A list of keypoints with timestamp.
	 */
	[[nodiscard]] dv::cvector<dv::TimedKeyPoint> runDetection(
		const InputType &input, size_t numPoints, const cv::Mat &mask = cv::Mat()) {
		dv::cvector<dv::TimedKeyPoint> result = detect(input, roiBuffered, mask);

		if (postProcessing == FeaturePostProcessing::AdaptiveNMS || postProcessing == FeaturePostProcessing::TopN) {
			std::sort(result.begin(), result.end(), [](const dv::TimedKeyPoint &a, const dv::TimedKeyPoint &b) {
				return a.response > b.response;
			});
		}

		switch (postProcessing) {
			case FeaturePostProcessing::AdaptiveNMS: {
				return resampler.resample(result, numPoints);
			}
			default:
				if (result.size() > numPoints) {
					result.resize(numPoints);
				}
				break;
		}
		return result;
	}

	/**
	 * Redetect new features and add them to already detected features. This function performs detection
	 * within masked region (if mask is non-empty), runs postprocessing and appends the additional features
	 * to the prior keypoint list.
	 * @param prior         A list of existing features.
	 * @param input         The input to the detector (events, images, etc.).
	 * @param numPoints     Number of total features after detection.
	 * @param mask          Detection mask.
	 */
	void runRedetection(dv::cvector<dv::TimedKeyPoint> &prior, const InputType &input, size_t numPoints,
		const cv::Mat &mask = cv::Mat()) {
		dv::cvector<dv::TimedKeyPoint> result = detect(input, roiBuffered, mask);

		if (result.empty()) {
			return;
		}

		if (postProcessing != FeaturePostProcessing::None) {
			std::sort(result.begin(), result.end(), [](const dv::TimedKeyPoint &a, const dv::TimedKeyPoint &b) {
				return a.response > b.response;
			});
		}

		switch (postProcessing) {
			case FeaturePostProcessing::None:
			case FeaturePostProcessing::TopN:
				if (result.size() > numPoints) {
					result.resize(numPoints);
				}
				break;
			case FeaturePostProcessing::AdaptiveNMS:
				result = resampler.resample(result, numPoints - prior.size());
			default:
				break;
		}

		std::move(result.begin(), result.end(), std::back_inserter(prior));
	}

	/**
	 * Get the type of post-processing.
	 * @sa FeatureDetectorBase::FeaturePostProcessing
	 * @return      Type of post-processing.
	 */
	[[nodiscard]] FeaturePostProcessing getPostProcessing() const {
		return postProcessing;
	}

	/**
	 * Set the type of post-processing.
	 * @sa FeatureDetectorBase::FeaturePostProcessing
	 * @param _postProcessing   Type of post-processing.
	 */
	void setPostProcessing(FeaturePostProcessing _postProcessing) {
		FeatureDetector::postProcessing = _postProcessing;
	}

	/**
	 * Get currently applied margin coefficient. Margin coefficient is multiplied by the width and height
	 * of  the image to calculate an adaptive border alongside the edges of image, where features should
	 * not be detected.
	 * @return      The margin coefficient.
	 */
	[[nodiscard]] float getMargin() const {
		return margin;
	}

	/**
	 * Set the margin coefficient. Margin coefficient is multiplied by the width and height
	 * of  the image to calculate an adaptive border alongside the edges of image, where features should
	 * not be detected.
	 * @param _margin       The margin coefficient
	 */
	void setMargin(float _margin) {
		if (_margin < 0.f || _margin > 0.5f) {
			throw std::invalid_argument(
				"Provided margin value (" + std::to_string(_margin) + ") is out of valid bounds [0; 0.5]");
		}
		margin      = _margin;
		roiBuffered = getMarginROI();
	}

	/**
	 * Check whether a point belongs to the ROI without the margins.
	 * @param point         Point to be checked
	 * @return              True if point belongs to the valid ROI, false otherwise.
	 */
	[[nodiscard]] bool isWithinROI(const cv::Point2f &point) const {
		return roiBuffered.contains(point);
	}

	/**
	 * Get configured image dimensions.
	 * @return              Image dimensions.
	 */
	[[nodiscard]] const cv::Size &getImageDimensions() const {
		return imageDimensions;
	}

private:
	FeaturePostProcessing postProcessing;

	float margin;

	cv::Size imageDimensions;

	cv::Rect roiBuffered;

	/**
	 * Container of the feature detector
	 */
	AlgorithmPtr detector;

	/**
	 * Class id counter, each new feature will be assigned on incremented class id.
	 */
	int classIdCounter = 0;
	KeyPointResampler resampler;

	/**
	 * The detection function to be implemented for feature detection. It should return a list
	 * of keypoints with a quality score, but it should *not* be ordered in any way. The sorting
	 * will be performed by the `runDetection` function as a postprocessing step.
	 * @param input     Input for the detector.
	 * @param roi       Region of interest where detection should be performed, the region is estimated
	 *                  using the margin configuration value.
	 * @param mask      Detection mask, can be empty. If non empty, the detection should be performed
	 *                  where mask value is non-zero.
	 * @return          A list of keypoint features with timestamp.
	 */
	[[nodiscard]] dv::cvector<dv::TimedKeyPoint> detect(
		const InputType &input, const cv::Rect &roi, const cv::Mat &mask) {
		dv::cvector<dv::TimedKeyPoint> output;

		if constexpr (concepts::OpenCVFeatureDetectorAlgorithm<Algorithm>) {
			std::vector<cv::KeyPoint> cvFeatures;

			if constexpr (std::is_same_v<InputType, dv::features::ImagePyramid>) {
				detector->detect(input.pyramid.front()(roi), cvFeatures, mask.empty() ? mask : mask(roi));
			}
			else if constexpr (std::is_same_v<InputType, dv::Frame>) {
				detector->detect(input.image(roi), cvFeatures, mask.empty() ? mask : mask(roi));
			}
			else if constexpr (dv::concepts::TimedImageContainer<InputType>) {
				detector->detect(input.image(roi), cvFeatures, mask.empty() ? mask : mask(roi));
			}
			else {
				static_assert(std::is_same_v<InputType, dv::features::ImagePyramid>
								  || std::is_same_v<InputType, dv::Frame> || std::is_same_v<InputType, dv::Frame>
								  || dv::concepts::TimedImageContainer<InputType>,
					"InputType is not supported with OpenCV feature detectors, should be dv::features::ImagePyramid "
					"or dv::Frame, or cv::mat, or satisfy dv::concepts::TimedImageContainer");
			}

			output = dv::data::fromCvKeypoints(cvFeatures, input.timestamp);
		}
		else {
			output = detector->detect(input, roi, mask.empty() ? mask : mask(roi));
		}

		for (auto &corner : output) {
			corner.class_id = classIdCounter++;

			if constexpr (concepts::OpenCVFeatureDetectorAlgorithm<Algorithm>) {
				// Coordinates within cv::Mat that represents an ROI are offset by the ROI top-left coordinates, so we
				// need to correct them
				corner.pt = dv::Point2f(corner.pt.x() + static_cast<float>(roiBuffered.x),
					corner.pt.y() + static_cast<float>(roiBuffered.y));

				// OpenCV feature detection algorithms don't provide a timestamp, so we need to add them
				corner.timestamp = input.timestamp;
			}
		}

		return output;
	}

	/**
	 * Calculate the region of interest with the margin coefficient. Margin is a coefficient of
	 * width / height, which should be used to ignore pixels near borders of the image.
	 * @return          Region of interest for detection of features.
	 */
	[[nodiscard]] cv::Rect getMarginROI() const {
		auto marginSizeX = static_cast<int>(std::ceil(static_cast<float>(imageDimensions.width) * margin));
		auto marginSizeY = static_cast<int>(std::ceil(static_cast<float>(imageDimensions.height) * margin));

		return {marginSizeX, marginSizeY, imageDimensions.width - (2 * marginSizeX),
			imageDimensions.height - (2 * marginSizeY)};
	}
};

} // namespace dv::features

#pragma once

#include "../camera/stereo_geometry.hpp"
#include "../core/filters.hpp"
#include "../core/frame.hpp"

#include <opencv2/imgproc.hpp>

namespace dv {

class SparseEventBlockMatcher {
protected:
	cv::Mat mLeftMask;
	cv::Mat mRightMask;

	dv::Frame mLeftFrame;
	dv::Frame mRightFrame;

	dv::EdgeMapAccumulator mLeftAcc;
	dv::EdgeMapAccumulator mRightAcc;

	cv::Size mWindowSize;
	cv::Size mHalfWindowSize;
	int32_t mMaxDisparity;
	int32_t mMinDisparity;
	float mMinScore;

	std::unique_ptr<dv::camera::StereoGeometry> mStereoGeometry = nullptr;

	template<dv::concepts::Coordinate2D InputPoint>
	[[nodiscard]] cv::Rect getPointRoi(const InputPoint &center, const int32_t offsetX, const int32_t stretchX) const {
		if constexpr (dv::concepts::Coordinate2DAccessors<InputPoint>) {
			return {(static_cast<int32_t>(center.x()) - mHalfWindowSize.width) + offsetX,
				static_cast<int32_t>(center.y()) - mHalfWindowSize.height, mWindowSize.width + stretchX,
				mWindowSize.height};
		}
		else if constexpr (dv::concepts::Coordinate2DMembers<InputPoint>) {
			return {(static_cast<int32_t>(center.x) - mHalfWindowSize.width) + offsetX,
				static_cast<int32_t>(center.y) - mHalfWindowSize.height, mWindowSize.width + stretchX,
				mWindowSize.height};
		}
	}

	void initializeMaskPoint(cv::Mat &mask, const int32_t offsetX, const int32_t stretchX, const cv::Point2i &coord,
		const std::optional<dv::camera::StereoGeometry::CameraPosition> cameraPosition = std::nullopt) const {
		const cv::Rect view(0, 0, mask.cols - 1, mask.rows - 1);
		if (cameraPosition.has_value()) {
			const cv::Rect position = getPointRoi(coord, offsetX, stretchX) & view;

			// Project the ROI in original input pixel space
			const cv::Point2i tl = mStereoGeometry->unmapPoint(*cameraPosition, position.tl());
			const cv::Point2i br = mStereoGeometry->unmapPoint(*cameraPosition, position.br());
			cv::rectangle(mask, cv::Rect(tl, br), cv::Scalar(255), cv::FILLED);
		}
		else {
			const cv::Rect position = getPointRoi(coord, offsetX, stretchX) & view;
			cv::rectangle(mask, position, cv::Scalar(255), cv::FILLED);
		}
	}

public:
	/**
	 * Structure containing disparity results for a point of interest.
	 */
	struct PixelDisparity {
		/// Point of interest coordinates, this will contain same coordinates that were passed into the algorithm.
		cv::Point2i coordinates;

		/// Holds true if the disparity match valid. False otherwise.
		bool valid;

		/// Pearson correlation value for the best matching block, if available. This value is in the range [-1.0; 1.0].
		/// Correlation value of -1.0 will mean that matched patch is an inverse of the original template patch, 1.0
		/// will be an equal match, 0.0 is no correlation. A positive value indicates a positive correlation between
		/// searched template patch and best match, which could be considered a good indication of a correct match.
		std::optional<float> correlation;

		/// Standard score (Z-score) for the match, if available. The score is the number of standard deviations
		/// the highest probability value is above the mean of all probabilities of the matching method.
		std::optional<float> score;

		/// Disparity value in pixels, if available. The value is in the range [minDisparity; maxDisparity].
		std::optional<int32_t> disparity;

		/// Coordinates of the matching template on the left (rectified) image space. Set to `std::nullopt` if the
		/// template coordinates are out-of-bounds.
		std::optional<cv::Point2i> templatePosition;

		/// Coordinates of the matched template on the right (rectified) image space. Set to `std::nulltopt` if a match
		/// cannot be reliably found, otherwise contains coordinates with highest correlation match on the right side
		/// rectified camera pixel space.
		std::optional<cv::Point2i> matchedPosition;

		/**
		 * Initialize the disparity structure.
		 * @param coordinates		Point of interest coordinates, this will contain same coordinates that were passed
		 * 							into the algorithm.
		 * @param valid				Holds true if the disparity match valid. False otherwise.
		 * @param correlation		Pearson correlation value for the best matching block, if available. This value is
		 * 							in the range [-1.0; 1.0].
		 * @param score				Matching score value, if available. This value is in the range [0.0; 1.0].
		 * @param disparity			Disparity value in pixels, if available. The value is in the
		 * 							range [minDisparity; maxDisparity].
		 * @param templatePosition 	Requested coordinate of interest point in the left (rectified) image pixel space.
		 * @param matchedPosition 	Best match coordinate on the right (rectified) image pixel space.
		 */
		PixelDisparity(const cv::Point2i &coordinates, const bool valid,
			const std::optional<float> correlation = std::nullopt, const std::optional<float> score = std::nullopt,
			const std::optional<int32_t> disparity             = std::nullopt,
			const std::optional<cv::Point2i> &templatePosition = std::nullopt,
			const std::optional<cv::Point2i> &matchedPosition  = std::nullopt) :
			coordinates(coordinates),
			valid(valid),
			correlation(correlation),
			score(score),
			disparity(disparity),
			templatePosition(templatePosition),
			matchedPosition(matchedPosition) {
		}
	};

	/**
	 * Initialize sparse event block matcher. This constructor initializes the matcher in non-rectified space, so
	 * for accurate results the event coordinates should be already rectified.
	 * @param resolution		Resolution of camera sensors. Assumes same resolution for left and right camera.
	 * @param windowSize		Matching window size.
	 * @param maxDisparity		Maximum disparity value.
	 * @param minDisparity		Minimum disparity value.
	 * @param minScore			Minimum matching score to consider matching valid.
	 */
	explicit SparseEventBlockMatcher(const cv::Size &resolution, const cv::Size &windowSize = cv::Size(24, 24),
		const int32_t maxDisparity = 40, const int32_t minDisparity = 0, const float minScore = 1.0f) :
		mLeftMask(resolution, CV_8UC1),
		mRightMask(resolution, CV_8UC1),
		mLeftAcc(resolution),
		mRightAcc(resolution),
		mWindowSize(windowSize),
		mHalfWindowSize(windowSize.width / 2, windowSize.height / 2),
		mMaxDisparity(maxDisparity),
		mMinDisparity(minDisparity),
		mMinScore(minScore) {
		if (mMaxDisparity <= mMinDisparity) {
			throw dv::exceptions::InvalidArgument<int32_t>(
				"Maximum disparity is less or equal to minimum disparity", mMaxDisparity);
		}
	}

	/**
	 * Initialize a sparse stereo block matcher with a calibrated stereo geometry. This allows event rectification
	 * while calculating the disparity.
	 * @param geometry			Stereo camera geometry.
	 * @param windowSize		Matching window size.
	 * @param maxDisparity		Maximum disparity value.
	 * @param minDisparity		Minimum disparity value.
	 * @param minScore			Minimum matching score to consider matching valid.
	 */
	explicit SparseEventBlockMatcher(std::unique_ptr<dv::camera::StereoGeometry> geometry,
		const cv::Size &windowSize = cv::Size(24, 24), const int32_t maxDisparity = 40, const int32_t minDisparity = 0,
		const float minScore = 1.0f) :
		mLeftMask(geometry->getLeftCameraGeometry().getResolution(), CV_8UC1),
		mRightMask(geometry->getRightCameraGeometry().getResolution(), CV_8UC1),
		mLeftAcc(geometry->getLeftCameraGeometry().getResolution()),
		mRightAcc(geometry->getRightCameraGeometry().getResolution()),
		mWindowSize(windowSize),
		mHalfWindowSize(windowSize.width / 2, windowSize.height / 2),
		mMaxDisparity(maxDisparity),
		mMinDisparity(minDisparity),
		mMinScore(minScore),
		mStereoGeometry(std::move(geometry)) {
		if (mMaxDisparity <= mMinDisparity) {
			throw dv::exceptions::InvalidArgument<int32_t>(
				"Maximum disparity is less or equal to minimum disparity", mMaxDisparity);
		}
		if (mStereoGeometry->getLeftCameraGeometry().getResolution()
			!= mStereoGeometry->getRightCameraGeometry().getResolution()) {
			throw dv::exceptions::RuntimeError(
				"Sparse block matching requires stereo camera setup with sensors of the same resolution.");
		}
	}

	/**
	 * Compute sparse disparity on given interest points. The events are accumulated sparsely only on the
	 * selected interest point regions. Returns a list of coordinates with their according disparity values,
	 * correlations and scores for each disparity match. If rectification is enabled, the returned disparity
	 * result will have `valid` flag set to false if the interest point coordinate lies outside of valid rectified
	 * pixel space.
	 *
	 * Input event has to be passed in synchronized batches, no time validation is performed during accumulation.
	 * @param left				Synchronised event batch from left camera.
	 * @param right				Synchronised event batch from right camera.
	 * @param interestPoints	List of interest coordinates in unrectified pixel space.
	 * @return					A list of disparity results for each input interest point.
	 */
	template<dv::concepts::Coordinate2DIterable InputPoints>
	[[nodiscard]] std::vector<PixelDisparity> computeDisparitySparse(
		const dv::EventStore &left, const dv::EventStore &right, const InputPoints &interestPoints) {
		using PointType      = dv::concepts::iterable_element_type<InputPoints>;
		using CameraPosition = dv::camera::StereoGeometry::CameraPosition;

		const int32_t numDisparities = mMaxDisparity - mMinDisparity;

		std::vector<std::pair<std::optional<cv::Point2i>, dv::concepts::iterable_element_type<InputPoints>>> points;
		points.reserve(interestPoints.size());

		mLeftMask  = 0;
		mRightMask = 0;

		if (mStereoGeometry) {
			for (const auto pt : interestPoints) {
				const auto remapped = mStereoGeometry->remapPoint(CameraPosition::Left, pt);
				points.push_back(std::make_pair(remapped, pt));
				if (remapped.has_value()) {
					initializeMaskPoint(mLeftMask, 0, 0, *remapped, camera::StereoGeometry::CameraPosition::Left);
					initializeMaskPoint(mRightMask, -mMaxDisparity, numDisparities, *remapped,
						camera::StereoGeometry::CameraPosition::Right);
				}
			}
		}
		else {
			for (const auto pt : interestPoints) {
				cv::Point2i casted;
				if constexpr (dv::concepts::Coordinate2DAccessors<PointType>) {
					casted = cv::Point2i(pt.x(), pt.y());
				}
				else {
					casted = cv::Point2i(pt.x, pt.y);
				}
				points.push_back(std::make_pair(casted, pt));
				initializeMaskPoint(mLeftMask, 0, 0, casted);
				initializeMaskPoint(mRightMask, -mMaxDisparity, numDisparities, casted);
			}
		}

		dv::EventMaskFilter maskFilterLeft(mLeftMask);
		maskFilterLeft.accept(left);
		mLeftAcc.accept(maskFilterLeft.generateEvents());
		mLeftFrame = mLeftAcc.generateFrame();

		dv::EventMaskFilter maskFilterRight(mRightMask);
		maskFilterRight.accept(right);
		mRightAcc.accept(maskFilterRight.generateEvents());
		mRightFrame = mRightAcc.generateFrame();

		if (mStereoGeometry) {
			mLeftFrame.image  = mStereoGeometry->remapImage(CameraPosition::Left, mLeftFrame.image);
			mRightFrame.image = mStereoGeometry->remapImage(CameraPosition::Right, mRightFrame.image);
		}

		std::vector<PixelDisparity> output;
		output.reserve(interestPoints.size());
		const cv::Rect leftRegion(0, 0, mLeftFrame.image.cols, mLeftFrame.image.rows);
		const cv::Rect rightRegion(0, 0, mRightFrame.image.cols, mRightFrame.image.rows);
		for (const auto &[pixel, point] : points) {
			cv::Point2i outputPoint;
			if constexpr (dv::concepts::Coordinate2DAccessors<PointType>) {
				outputPoint = cv::Point2i(point.x(), point.y());
			}
			else {
				outputPoint = cv::Point2i(point.x, point.y);
			}
			if (!pixel.has_value()) {
				output.emplace_back(outputPoint, false);
				continue;
			}

			// Here we select the region of interest on left image
			const cv::Rect templateROI = getPointRoi(*pixel, 0, 0);
			// Limit the region is not outside of the valid image space.
			const cv::Mat template_ = mLeftFrame.image(templateROI & leftRegion);

			// Here we select the search space for template matching on the right image. Template is selected
			// by going for left-most max-disparity pixel location and to the right by the number of disparities
			// from configuration. This select a horizontal template matching space.
			const cv::Rect imageROI = getPointRoi(*pixel, -mMaxDisparity, numDisparities);
			// Limit the region is not outside of the valid image space.
			const cv::Mat image = mRightFrame.image(imageROI & rightRegion);

			// Reject patches that are too small
			if (image.cols - template_.cols <= 1) {
				output.emplace_back(outputPoint, false);
				continue;
			}

			// Run template matching
			cv::Mat result;
			cv::matchTemplate(image, template_, result, cv::TemplateMatchModes::TM_CCOEFF_NORMED);
			Eigen::VectorXf correlations;
			cv::cv2eigen(result.reshape(0, result.cols), correlations);

			// Softmax score
			auto shifted                = correlations.array() * 20.f;
			auto exponents              = shifted.exp();
			const auto probabilities    = (exponents / exponents.sum());
			const float meanProbability = probabilities.mean();
			const float stddev          = std::sqrt(
                (probabilities - meanProbability).square().sum() / (static_cast<float>(probabilities.size())));

			// Disparity value at maximum probability
			int32_t disparity;
			const float probability = probabilities.maxCoeff(&disparity);

			const float correlation = correlations(disparity);
			// If standard deviation close to zero, override score to zero since this assigns high zscores on
			// noise
			const float score = stddev < 0.1f ? 0.f : (probability - meanProbability) / stddev;

			// Actual disparity value corrected to the search space configuration
			const int32_t disparityValue = (image.cols - template_.cols) - disparity + mMinDisparity;
			output.emplace_back(outputPoint, score >= mMinScore, correlation, score, disparityValue, *pixel,
				cv::Point2i(pixel->x - disparityValue, pixel->y));
		}

		return output;
	}

	/**
	 * Get the left camera image mask. The algorithm only accumulates the frames where actual matching is going to
	 * happen. The mask will contain non-zero pixel values where accumulation needs to happen.
	 * @return	Interest region mask for left camera.
	 */
	[[nodiscard]] const cv::Mat &getLeftMask() const {
		return mLeftMask;
	}

	/**
	 * Get the right camera image mask. The algorithm only accumulates the frames where actual matching is going to
	 * happen. The mask will contain non-zero pixel values where accumulation needs to happen.
	 * @return	Interest region mask for right camera.
	 */
	[[nodiscard]] const cv::Mat &getRightMask() const {
		return mRightMask;
	}

	/**
	 * Get the latest accumulated left frame.
	 * @return	Accumulated image of the left camera from last disparity computation step.
	 */
	[[nodiscard]] dv::Frame getLeftFrame() const {
		return mLeftFrame;
	}

	/**
	 * Get the latest accumulated right frame.
	 * @return	Accumulated image of the right camera from last disparity computation step.
	 */
	[[nodiscard]] dv::Frame getRightFrame() const {
		return mRightFrame;
	}

	/**
	 * Get matching window size.
	 * @return		Currently configured matching window size.
	 */
	[[nodiscard]] const cv::Size &getWindowSize() const {
		return mWindowSize;
	}

	/**
	 * Set matching window size. This is the size of cropped template image that is matched along the epipolar
	 * line of the stereo geometry.
	 * @param windowSize	New matching window size.
	 */
	void setWindowSize(const cv::Size &windowSize) {
		mWindowSize     = windowSize;
		mHalfWindowSize = cv::Size(windowSize.width / 2, windowSize.height / 2);
	}

	/**
	 * Get maximum disparity value.
	 * @return		Currently configured maximum disparity value.
	 */
	[[nodiscard]] int32_t getMaxDisparity() const {
		return mMaxDisparity;
	}

	/**
	 * Set maximum measured disparity. This parameter limits the matching space in pixels on the right camera image.
	 * @param maxDisparity		New maximum disparity value.
	 */
	void setMaxDisparity(const int32_t maxDisparity) {
		mMaxDisparity = maxDisparity;
	}

	/**
	 * Get minimum disparity value.
	 * @return		Currently configured minimum disparity value.
	 */
	[[nodiscard]] int32_t getMinDisparity() const {
		return mMinDisparity;
	}

	/**
	 * Set minimum measured disparity. This parameter limits the matching space in pixels on the right camera image.
	 * @param minDisparity		New minimum disparity value.
	 */
	void setMinDisparity(const int32_t minDisparity) {
		mMinDisparity = minDisparity;
	}

	/**
	 * Get minimum matching score value.
	 * @return	Currently configured minimum matching score value.
	 */
	[[nodiscard]] float getMinScore() const {
		return mMinScore;
	}

	/**
	 * Set minimum matching score value to consider the matching valid. If matching score is below this threshold,
	 * the value for a point will be set to an invalid value and `valid` boolean to false.
	 *
	 * Score is calculated by applying softmax function on the discrete distribution of correlation values
	 * from matching the template left patch on the epipolar line of the right camera image. This retrieves
	 * the probability mass function of the correlations. The best match is found by finding the max probability
	 * value and score is calculated for the best match by computing z-score over the probabilities.
	 *
	 * @param minimumScore		New minimum score value.
	 */
	void setMinScore(const float minimumScore) {
		mMinScore = minimumScore;
	}
};

} // namespace dv

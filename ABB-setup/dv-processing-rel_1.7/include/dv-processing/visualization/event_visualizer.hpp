#pragma once

#include "../core/core.hpp"
#include "../core/utils.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "colors.hpp"

namespace dv::visualization {

/**
 * EventVisualizer class implements simple color-coded representation of events. It applies
 * certain colors where positive or negative polarity events are registered.
 */
class EventVisualizer {
private:
	const cv::Size resolution;

	cv::Vec3b backgroundColor;
	cv::Vec3b positiveColor;
	cv::Vec3b negativeColor;

public:
	/**
	 * Initialize event visualizer.
	 * @param resolution 		Resolution of incoming events.
	 * @param backgroundColor	Background color.
	 * @param positiveColor		Color applied to positive polarity events.
	 * @param negativeColor		Color applied to negative polarity events.
	 */
	explicit EventVisualizer(const cv::Size &resolution, const cv::Scalar &backgroundColor = colors::white,
		const cv::Scalar &positiveColor = colors::iniBlue, const cv::Scalar &negativeColor = colors::darkGrey) :
		resolution(resolution) {
		setBackgroundColor(backgroundColor);
		setPositiveColor(positiveColor);
		setNegativeColor(negativeColor);
	}

	/**
	 * Generate a preview image from an event store.
	 * @param events	Input events.
	 * @return			Colored preview image of given events.
	 */
	[[nodiscard]] cv::Mat generateImage(const dv::EventStore &events) const {
		cv::Mat image(resolution, CV_8UC3, backgroundColor);

		generateImage(events, image);

		return image;
	}

	/**
	 * Generate a preview image from an event store.
	 * @param events Input events.
	 * @param background Image to draw the events on. The pixels type has to be 3-channel 8-bit unsigned integer (BGR).
	 */
	void generateImage(const dv::EventStore &events, cv::Mat &background) const {
		if (background.type() != CV_8UC3) {
			throw dv::exceptions::RuntimeError(
				"Visualizer requires 3-channel 8-bit unsigned integer image for background");
		}

		if (background.size() != resolution) {
			throw dv::exceptions::RuntimeError("Incompatible resolution image for visualizer");
		}

		for (const auto &event : events) {
			dv::runtime_assert(
				dv::isWithinDimensions(event, resolution), "Received an event outside of valid visualizer resolution");
			background.at<cv::Vec3b>(event.y(), event.x()) = event.polarity() ? positiveColor : negativeColor;
		}
	}

	/**
	 * Get currently configured background color.
	 * @return	Background color.
	 */
	[[nodiscard]] cv::Scalar getBackgroundColor() const {
		return backgroundColor;
	}

	/**
	 * Set new background color.
	 * @param backgroundColor_	New background color.
	 */
	void setBackgroundColor(const cv::Scalar &backgroundColor_) {
		backgroundColor = cv::Vec3b(static_cast<uint8_t>(backgroundColor_(0)),
			static_cast<uint8_t>(backgroundColor_(1)), static_cast<uint8_t>(backgroundColor_(2)));
	}

	/**
	 * Get currently configured positive polarity color.
	 * @return	Positive polarity color.
	 */
	[[nodiscard]] cv::Scalar getPositiveColor() const {
		return positiveColor;
	}

	/**
	 * Set new positive polarity color.
	 * @param positiveColor_	New positive polarity color.
	 */
	void setPositiveColor(const cv::Scalar &positiveColor_) {
		positiveColor = cv::Vec3b(static_cast<uint8_t>(positiveColor_(0)), static_cast<uint8_t>(positiveColor_(1)),
			static_cast<uint8_t>(positiveColor_(2)));
	}

	/**
	 * Get negative polarity color.
	 * @return	Negative polarity color.
	 */
	[[nodiscard]] cv::Scalar getNegativeColor() const {
		return negativeColor;
	}

	/**
	 * Set new negative polarity color.
	 * @param negativeColor_	New negative polarity color.
	 */
	void setNegativeColor(const cv::Scalar &negativeColor_) {
		negativeColor = cv::Vec3b(static_cast<uint8_t>(negativeColor_(0)), static_cast<uint8_t>(negativeColor_(1)),
			static_cast<uint8_t>(negativeColor_(2)));
	}
};

} // namespace dv::visualization

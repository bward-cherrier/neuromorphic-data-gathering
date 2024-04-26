#pragma once

#include <opencv2/core.hpp>

namespace dv::visualization::colors {

static const cv::Scalar black    = cv::Scalar(0, 0, 0);
static const cv::Scalar white    = cv::Scalar(255, 255, 255);
static const cv::Scalar red      = cv::Scalar(0, 0, 255);
static const cv::Scalar lime     = cv::Scalar(0, 255, 0);
static const cv::Scalar blue     = cv::Scalar(255, 0, 0);
static const cv::Scalar yellow   = cv::Scalar(0, 255, 255);
static const cv::Scalar cyan     = cv::Scalar(255, 255, 0);
static const cv::Scalar magenta  = cv::Scalar(255, 0, 255);
static const cv::Scalar silver   = cv::Scalar(192, 192, 192);
static const cv::Scalar gray     = cv::Scalar(128, 128, 128);
static const cv::Scalar navy     = cv::Scalar(128, 0, 0);
static const cv::Scalar green    = cv::Scalar(0, 128, 0);
static const cv::Scalar iniBlue  = cv::Scalar(183, 93, 0);
static const cv::Scalar darkGrey = cv::Scalar(43, 43, 43);

[[deprecated("iniblue is deprecated, use iniBlue instead")]] static const auto iniblue    = iniBlue;
[[deprecated("darkgrey is deprecated, use darkGrey instead")]] static const auto darkgrey = darkGrey;

static const std::vector<cv::Scalar> neonPalette
	= {cv::Scalar(255, 111, 0), cv::Scalar(239, 244, 19), cv::Scalar(0, 255, 104), cv::Scalar(0, 255, 250),
		cv::Scalar(0, 191, 255), cv::Scalar(0, 191, 255), cv::Scalar(92, 0, 255)};

[[nodiscard]] inline cv::Scalar someNeonColor(const int32_t someNumber) {
	return neonPalette[static_cast<size_t>(someNumber) % neonPalette.size()];
}

} // namespace dv::visualization::colors

#pragma once

#include "../core/core.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "../visualization/colors.hpp"

#include <opencv2/imgproc.hpp>

#include <random>

namespace dv::data::generate {

/**
 * Generate a sample image (single channel 8-bit unsigned integer) containing a few gray rectangles
 * in a black background.
 * @param resolution 	Resolution of the output image.
 * @return 				Generated image.
 */
[[nodiscard]] inline cv::Mat sampleImage(const cv::Size &resolution) {
	cv::Mat image = cv::Mat::zeros(resolution, CV_8UC1);
	cv::Point point(resolution.width / 10, resolution.height / 10);
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	point *= 2;
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	point += point;
	cv::rectangle(image, point, point + point, cv::Scalar(128), 5);
	return image;
}

/**
 * Generate events along a line between two given end-points.
 * @param timestamp 	Fixed timestamp assigned for all events.
 * @param a 			Starting point.
 * @param b 			Ending point.
 * @param steps 		Number of events generated for the line. If zero is provided, the function uses euclidean
 * 						distance between the points.
 * @return				A batch of event along the line.
 */
[[nodiscard]] inline dv::EventStore eventLine(
	const int64_t timestamp, const cv::Point &a, const cv::Point &b, size_t steps = 0) {
	dv::EventStore store;
	if (steps == 0) {
		steps = static_cast<size_t>(cv::norm(b - a));
	}
	auto totalSteps = static_cast<double>(steps);
	double diffX    = b.x - a.x;
	double diffY    = b.y - a.y;
	for (size_t i = 0; i < steps; i++) {
		double alpha = static_cast<double>(i) / totalSteps;
		store.emplace_back(timestamp, a.x + (alpha * diffX), a.y + (alpha * diffY), true);
	}
	return store;
}

/**
 * Generate events along a rectangle edges between two given top-left and bottom right points.
 * @param timestamp Fixed timestamp assigned for all events.
 * @param tl 		Top left coordinate of the rectangle.
 * @param br 		Bottom right coordinate of the rectangle.
 * @return 			Event batch containing events at the edges of a given rectangle.
 */
[[nodiscard]] inline dv::EventStore eventRectangle(const int64_t timestamp, const cv::Point &tl, const cv::Point &br) {
	dv::EventStore rectangle;
	// Generate an approximate amount of pixels for a full line
	rectangle.add(eventLine(timestamp, tl, cv::Point(tl.x, br.y)));
	rectangle.add(eventLine(timestamp, cv::Point(tl.x, br.y), br));
	rectangle.add(eventLine(timestamp, br, cv::Point(br.x, tl.y)));
	rectangle.add(eventLine(timestamp, cv::Point(br.x, tl.y), tl));
	return rectangle;
}

/**
 * Generate an event test set that contains event for a few intersecting rectangle edges.
 * @param timestamp 	Fixed timestamp assigned for all events.
 * @param resolution 	Expected resolution limits for the events.
 * @return 				Generated event batch.
 */
[[nodiscard]] inline dv::EventStore eventTestSet(const int64_t timestamp, const cv::Size &resolution) {
	dv::EventStore points;
	cv::Point offset(1, 1);
	cv::Point point(resolution.width / 10, resolution.height / 10);
	points.add(eventRectangle(timestamp, point, point + point));
	points.add(eventRectangle(timestamp, point + offset, (point + point)));
	points.add(eventRectangle(timestamp, point - offset, (point + point)));
	point *= 2;
	points.add(eventRectangle(timestamp, point, point + point));
	points.add(eventRectangle(timestamp, point + offset, (point + point)));
	points.add(eventRectangle(timestamp, point - offset, (point + point)));
	point += point;
	points.add(eventRectangle(timestamp, point, point + point));
	points.add(eventRectangle(timestamp, point + offset, (point + point)));
	points.add(eventRectangle(timestamp, point - offset, (point + point)));
	return points;
}

/**
 * Generate a batch of uniformly distributed set of event within the given resolution.
 * @param timestamp 	Fixed timestamp assigned for all events.
 * @param resolution 	Resolution limits.
 * @param count 		Number of events.
 * @param seed 			Seed for the RNG.
 * @return 				Generated event batch.
 */
[[nodiscard]] inline dv::EventStore uniformlyDistributedEvents(
	const int64_t timestamp, const cv::Size &resolution, const size_t count, const uint64_t seed = 0) {
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	dv::EventStore store;
	for (size_t i = 0; i < count; i++) {
		store.emplace_back(timestamp, static_cast<int16_t>(distribution(generator) * resolution.width),
			static_cast<int16_t>(distribution(generator) * resolution.height), distribution(generator) > 0.5);
	}
	return store;
}

/**
 * Generate events normally distributed around a given center coordinates with given standard deviation.
 * @param timestamp Timestamp to be assigned to the generated events
 * @param center Center coordinates
 * @param stddev Standard deviation for each of the axes
 * @param count Number of events to generate
 * @param seed Seed for the RNG
 * @return Set of normally distributed events
 */
[[nodiscard]] inline dv::EventStore normallyDistributedEvents(const int64_t timestamp, const dv::Point2f &center,
	const dv::Point2f &stddev, const size_t count, const uint64_t seed = 0) {
	std::default_random_engine generator(seed);
	std::normal_distribution<float> distributionX(0.0, stddev.x());
	std::normal_distribution<float> distributionY(0.0, stddev.y());

	dv::EventStore store;
	for (size_t i = 0; i < count; i++) {
		store.emplace_back(timestamp, static_cast<int16_t>(distributionX(generator) + center.x()),
			static_cast<int16_t>(distributionX(generator) + center.y()), distributionX(generator) > 0.f);
	}
	return store;
}

/**
 * Generate a batch of uniformly distributed (in pixel-space) randomly generated events. The timestamps
 * are generated by monotonically increasing the timestamp within the time duration.
 * @param startTime 	Start timestamp in microseconds.
 * @param duration 		Duration of the generated data.
 * @param resolution 	Pixel space resolution.
 * @param count 		Number of output events.
 * @param seed 			Seed for the RNG.
 * @return 				Generated event batch.
 */
[[nodiscard]] inline dv::EventStore uniformEventsWithinTimeRange(const int64_t startTime, const dv::Duration duration,
	const cv::Size &resolution, const int64_t count, const uint64_t seed = 0) {
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const int64_t timestampIncrement = duration.count() / static_cast<int64_t>(count);

	dv::EventStore store;
	for (int64_t i = 0; i < count; i++) {
		store.emplace_back(startTime + (i * timestampIncrement),
			static_cast<int16_t>(distribution(generator) * resolution.width),
			static_cast<int16_t>(distribution(generator) * resolution.height), distribution(generator) > 0.5);
	}
	return store;
}

/**
 * Generate a DV logo using simple drawing methods. Generates in color or grayscale.
 * @param size		Output dimensions of the drawing
 * @param colored	Colored output (CV_8UC3) if true, or grayscale (CV_8UC1) otherwise.
 * @return			Image containing DV logo.
 */
[[nodiscard]] inline cv::Mat dvLogo(const cv::Size &size, const bool colored = true,
	const cv::Scalar &bgColor = dv::visualization::colors::white,
	const cv::Scalar &pColor  = dv::visualization::colors::iniBlue,
	const cv::Scalar &nColor  = dv::visualization::colors::darkGrey) {
	cv::Mat image(size, colored ? CV_8UC3 : CV_8UC1, bgColor);
	const double radius = std::min(size.width, size.height) / 2.0;
	cv::circle(image, cv::Point2d(radius, radius), static_cast<int>(radius), nColor, cv::FILLED);
	cv::putText(image, "DV", cv::Point2d(0.3 * radius, 1.35 * radius), cv::FONT_HERSHEY_SIMPLEX, radius * 0.035, pColor,
		static_cast<int>(radius * 0.08), cv::FILLED);
	return image;
}

/**
 * Convert an image into event by matching pixel intensities. The algorithm will match all pixel values available
 * in the and match against positive and negative pixel intensity values, according events are going to be added
 * into the output event store. Other pixel intensity values are ignored.
 * @param image 	Input image for conversion
 * @param positive 	Pixel brightness intensity value to consider the pixel to generate a positive polarity event.
 * @param negative 	Pixel brightness intensity value to consider the pixel to generate a negative polarity event.
 * @return 			Generated events.
 */
[[nodiscard]] inline dv::EventStore imageToEvents(
	const int64_t timestamp, const cv::Mat &image, const uint8_t positive, const uint8_t negative) {
	dv::EventStore output;

	if (image.channels() != 1 || image.depth() != CV_8U) {
		throw dv::exceptions::RuntimeError("Method requires an image of type CV_8UC1.");
	}

	uint8_t *pixel = image.data;
	for (int j = 0; j < image.rows; ++j) {
		for (int i = 0; i < image.cols; ++i, ++pixel) {
			if (*pixel == positive) {
				output.emplace_back(timestamp, i, j, true);
			}
			else if (*pixel == negative) {
				output.emplace_back(timestamp, i, j, false);
			}
		}
	}

	return output;
}

/**
 * Generate a DV logo using simple drawing methods. Generates negative polarity events on the pixels where logo has
 * dark pixels and positive polarity events where pixels have brighter events.
 * @param timestamp 	Timestamp assigned to each generated event.
 * @param resolution 	Resolution of the events.
 * @return 				Events that can be accumulated / visualized to generate a logo of DV.
 */
[[nodiscard]] inline dv::EventStore dvLogoAsEvents(const int64_t timestamp, const cv::Size &resolution) {
	const cv::Mat logo = dv::data::generate::dvLogo(resolution, false, cv::Scalar(0), cv::Scalar(1), cv::Scalar(2));
	return imageToEvents(timestamp, logo, 1, 2);
}

/**
 * Generate an IMU measurement that measures a camera being on a stable and level surface. All measurement values
 * are going to be zero, except for Y axis of accelerometer, it will measure -1.0G.
 * @param timestamp	Timestamp to be assigned to the measurement.
 * @return			Generated IMU measurement.
 */
[[nodiscard]] inline dv::IMU levelImuMeasurement(const int64_t timestamp) {
	return {timestamp, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
}

/**
 * Apply noise to imu measurements (accelerometer and gyroscope). The noise is modelled as a normal distribution
 * with 0 mean and given standard deviation. The modelled noise is added to the given measurement and return a new
 * dv::IMU structure with added noise.
 * @param measurement 			IMU measurement to add noise to.
 * @param accelerometerStddev 	Accelerometer noise standard deviation.
 * @param gyroscopeStddev 		Gyroscope noise standard deviation.
 * @param seed 					Seed for the RNG.
 * @return						Generated measurement with added noise.
 */
[[nodiscard]] inline dv::IMU addNoiseToImu(
	const dv::IMU &measurement, const float accelerometerStddev, const float gyroscopeStddev, const uint64_t seed = 0) {
	std::default_random_engine generator(seed);
	std::normal_distribution<float> accDistribution(0.0, accelerometerStddev);
	std::normal_distribution<float> gyroDistribution(0.0, gyroscopeStddev);

	return {measurement.timestamp, measurement.temperature, measurement.accelerometerX + accDistribution(generator),
		measurement.accelerometerY + accDistribution(generator),
		measurement.accelerometerZ + accDistribution(generator), measurement.gyroscopeX + gyroDistribution(generator),
		measurement.gyroscopeY + gyroDistribution(generator), measurement.gyroscopeZ + gyroDistribution(generator),
		measurement.magnetometerX, measurement.magnetometerY, measurement.magnetometerZ};
}

/**
 * Generate an IMU measurement that measures a camera being on a stable and level surface with additional measurement
 * noise. The noise is modelled as a normal distribution with 0 mean and given standard deviation.
 * @param timestamp				Timestamp to be assigned to the measurement.
 * @param accelerometerStddev 	Accelerometer noise standard deviation.
 * @param gyroscopeStddev 		Gyroscope noise standard deviation.
 * @param seed 					Seed for the RNG.
 * @return						Generated IMU measurement.
 */
[[nodiscard]] inline dv::IMU levelImuWithNoise(const int64_t timestamp, const float accelerometerStddev = 0.1f,
	const float gyroscopeStddev = 0.01f, const uint64_t seed = 0) {
	const auto imu = levelImuMeasurement(timestamp);
	return addNoiseToImu(imu, accelerometerStddev, gyroscopeStddev, seed);
}

} // namespace dv::data::generate

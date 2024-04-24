#include "../../include/dv-processing/data/utilities.hpp"
#include "../../include/dv-processing/features/keypoint_resampler.hpp"

#include "boost/ut.hpp"

#include <opencv2/opencv.hpp>

#include <random>

int main() {
	using namespace boost::ut;

	"simple_resampling"_test = [] {
		cv::Size resolution(100, 100);
		dv::features::KeyPointResampler resampler(resolution);

		dv::cvector<cv::Point2f> keypoints;

		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution(0.0, 1.0);

		for (size_t i = 0; i < 1000; i++) {
			keypoints.emplace_back(
				distribution(generator) * resolution.width, distribution(generator) * resolution.height);
		}

		size_t expectedAmount = 100;
		auto resampled        = resampler.resample(keypoints, expectedAmount);

		auto minResamples = static_cast<size_t>(static_cast<float>(expectedAmount) * (1.f - resampler.getTolerance()));
		auto maxResamples = static_cast<size_t>(static_cast<float>(expectedAmount) * (1.f + resampler.getTolerance()));

		expect(ge(resampled.size(), minResamples));
		expect(le(resampled.size(), maxResamples));

		resampled = resampler.resample(keypoints, 0);
		expect(eq(resampled.size(), 0));

		resampled = resampler.resample(keypoints, keypoints.size() * 2);
		expect(eq(resampled.size(), keypoints.size()));
	};

	"empty_vector"_test = [] {
		cv::Size resolution(100, 100);
		dv::features::KeyPointResampler resampler(resolution);

		dv::cvector<cv::Point2f> keypoints;

		size_t expectedAmount = 100;
		auto resampled        = resampler.resample(keypoints, expectedAmount);

		expect(eq(resampled.size(), 0));
	};

	"different_types"_test = [] {
		cv::Size resolution(100, 100);
		dv::features::KeyPointResampler resampler(resolution);

		std::vector<cv::KeyPoint> keypoints;
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0.f, 1.f);

		for (size_t i = 0; i < 1000; i++) {
			keypoints.emplace_back(distribution(generator) * static_cast<float>(resolution.width),
				distribution(generator) * static_cast<float>(resolution.height), 1.f);
		}

		size_t expectedAmount = 200;
		resampler.setTolerance(0.2f);
		auto resampled = resampler.resample(keypoints, expectedAmount);

		auto minResamples = static_cast<size_t>(static_cast<float>(expectedAmount) * (1.f - resampler.getTolerance()));
		auto maxResamples = static_cast<size_t>(static_cast<float>(expectedAmount) * (1.f + resampler.getTolerance()));

		expect(ge(resampled.size(), minResamples));
		expect(le(resampled.size(), maxResamples));

		// Mixing types should be fine for the resampler
		auto dvKeyPoints = dv::data::fromCvKeypoints(keypoints);
		auto dvResampled = resampler.resample(dvKeyPoints, expectedAmount);

		expect(ge(dvResampled.size(), minResamples));
		expect(le(dvResampled.size(), maxResamples));
	};
}

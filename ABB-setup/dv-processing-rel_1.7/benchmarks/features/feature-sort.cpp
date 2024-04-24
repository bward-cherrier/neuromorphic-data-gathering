//
// Created by rokas on 03.09.21.
//

#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/features/feature_detector.hpp"

#include <benchmark/benchmark.h>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>

dv::cvector<dv::TimedKeyPoint> loadFeatures() {
	namespace fs = std::filesystem;

	if (!fs::exists("../tests/samples/data")) {
		return {};
	}

	std::string image_path = "../tests/samples/data/home.jpg";
	cv::Mat img            = cv::imread(image_path, cv::IMREAD_COLOR);

	dv::features::ImageFeatureDetector detector(img.size(), cv::FastFeatureDetector::create(10),
		dv::features::ImageFeatureDetector::FeaturePostProcessing::None, 0.f);

	return detector.runDetection(dv::Frame(0, img), 10000);
}

void bmSortDvTimedKeyPoints(benchmark::State &state) {
	const uint32_t numSamples = state.range(0);

	dv::EventStore events;

	auto features = loadFeatures();
	features.resize(numSamples);

	for (auto _ : state) {
		std::sort(features.begin(), features.end(), [](const dv::TimedKeyPoint &a, const dv::TimedKeyPoint &b) -> bool {
			return a.response > b.response;
		});
	}

	state.SetComplexityN(state.range(0));
}

void bmSortStdTimedKeyPoints(benchmark::State &state) {
	const uint32_t numSamples = state.range(0);

	dv::EventStore events;

	auto features = loadFeatures();
	features.resize(numSamples);

	std::vector<dv::TimedKeyPoint> stdFeatures(features.begin(), features.end());

	for (auto _ : state) {
		std::sort(
			stdFeatures.begin(), stdFeatures.end(), [](const dv::TimedKeyPoint &a, const dv::TimedKeyPoint &b) -> bool {
				return a.response > b.response;
			});
	}

	state.SetComplexityN(state.range(0));
}

int main(int argc, char **argv) {
	static constexpr auto start = 1000;
	static constexpr auto stop  = 10000;
	static constexpr auto step  = (stop - start) / 10.0;

	BENCHMARK(bmSortDvTimedKeyPoints)->DenseRange(start, stop, step)->Complexity(benchmark::oN);
	BENCHMARK(bmSortStdTimedKeyPoints)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

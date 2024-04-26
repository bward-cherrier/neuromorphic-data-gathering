#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/depth/semi_dense_stereo_matcher.hpp"
#include "../../include/dv-processing/visualization/event_visualizer.hpp"

#include <benchmark/benchmark.h>

#include <random>

static const size_t eventCount     = 100000;
static const size_t iterationCount = 1500;
static const cv::Size resolution(640, 480);

static void bmDenseMatcherFrameRate(benchmark::State &state) {
	dv::SemiDenseStereoMatcher matcher(resolution, resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	size_t counter = 0;
	cv::Mat disparity;
	for (auto _ : state) {
		disparity = matcher.computeDisparity(store, store);
		counter++;
	}
	state.SetItemsProcessed(counter);
	state.SetLabel("frames");
}

static void bmDenseMatcherEventRate(benchmark::State &state) {
	dv::SemiDenseStereoMatcher matcher(resolution, resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	size_t counter = 0;
	cv::Mat disparity;
	for (auto _ : state) {
		disparity = matcher.computeDisparity(store, store);
		counter   += store.size() * 2;
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

int main(int argc, char **argv) {
	BENCHMARK(bmDenseMatcherFrameRate);
	BENCHMARK(bmDenseMatcherEventRate);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

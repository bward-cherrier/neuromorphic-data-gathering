#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/depth/sparse_event_block_matcher.hpp"
#include "../../include/dv-processing/visualization/event_visualizer.hpp"

#include <benchmark/benchmark.h>

#include <random>

static const size_t pointCount     = 100;
static const size_t iterationCount = 1500;
static const cv::Size resolution(640, 480);

static void bmDenseMatcherFrameRate(benchmark::State &state) {
	const size_t eventCount = state.range(0);

	dv::SparseEventBlockMatcher matcher(resolution, resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}

	std::vector<cv::Point2i> pointsOfInterest;
	for (const auto ev : store.slice(0, pointCount)) {
		pointsOfInterest.emplace_back(ev.x(), ev.y());
	}

	size_t counter = 0;
	for (auto _ : state) {
		auto disparity = matcher.computeDisparitySparse(store, store, pointsOfInterest);
		counter++;
	}
	state.SetItemsProcessed(counter);
	state.SetLabel("frames");
}

static void bmDenseMatcherEventRate(benchmark::State &state) {
	const size_t eventCount = state.range(0);
	dv::SparseEventBlockMatcher matcher(resolution, resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	std::vector<cv::Point2i> pointsOfInterest;
	for (const auto ev : store.slice(0, pointCount)) {
		pointsOfInterest.emplace_back(ev.x(), ev.y());
	}

	size_t counter = 0;
	for (auto _ : state) {
		auto disparity = matcher.computeDisparitySparse(store, store, pointsOfInterest);
		counter        += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmDenseMatcherDisparityRate(benchmark::State &state) {
	const size_t eventCount = state.range(0);
	dv::SparseEventBlockMatcher matcher(resolution, resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	std::vector<cv::Point2i> pointsOfInterest;
	for (const auto ev : store.slice(0, pointCount)) {
		pointsOfInterest.emplace_back(ev.x(), ev.y());
	}

	size_t counter = 0;
	for (auto _ : state) {
		auto disparity = matcher.computeDisparitySparse(store, store, pointsOfInterest);
		counter        += pointsOfInterest.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("disparities");
}

int main(int argc, char **argv) {
	BENCHMARK(bmDenseMatcherFrameRate)->DenseRange(5'000, 30'000, 5'000)->Complexity();
	BENCHMARK(bmDenseMatcherEventRate)->DenseRange(5'000, 30'000, 5'000)->Complexity();
	BENCHMARK(bmDenseMatcherDisparityRate)->DenseRange(5'000, 30'000, 5'000)->Complexity();

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/noise/background_activity_noise_filter.hpp"
#include "../../include/dv-processing/noise/fast_decay_noise_filter.hpp"

#include <benchmark/benchmark.h>
#include <opencv2/imgproc.hpp>

#include <random>

static void bmFastDecayNoiseFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	dv::noise::FastDecayNoiseFilter filter(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmRefractoryPeriodFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	dv::RefractoryPeriodFilter filter(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmBackgroundActivityNoiseFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	dv::noise::BackgroundActivityNoiseFilter filter(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmPolarityFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	dv::EventPolarityFilter filter(true);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmRegionFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	// Half of resolution
	dv::EventRegionFilter filter(cv::Rect(0, 0, 320, 480));
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmMaskFilter(benchmark::State &state) {
	cv::Size resolution(640, 480);

	// Half of resolution
	cv::Rect roi(0, 0, 320, 480);
	cv::Mat mask(resolution, CV_8UC1, cv::Scalar(0));
	cv::rectangle(mask, roi, cv::Scalar(255), cv::FILLED);

	dv::EventMaskFilter filter(mask);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmThreeFiltersNoChain(benchmark::State &state) {
	cv::Size resolution(640, 480);

	// Half of resolution
	cv::Rect roi(0, 0, 320, 480);
	cv::Mat mask(resolution, CV_8UC1, cv::Scalar(0));
	cv::rectangle(mask, roi, cv::Scalar(255), cv::FILLED);

	dv::EventMaskFilter maskFilter(mask);
	dv::EventPolarityFilter polarityFilter(true);
	dv::noise::BackgroundActivityNoiseFilter noiseFilter(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		maskFilter.accept(store);
		polarityFilter.accept(maskFilter.generateEvents());
		noiseFilter.accept(polarityFilter.generateEvents());
		filtered += noiseFilter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmThreeFiltersWithChain(benchmark::State &state) {
	cv::Size resolution(640, 480);

	// Half of resolution
	cv::Rect roi(0, 0, 320, 480);
	cv::Mat mask(resolution, CV_8UC1, cv::Scalar(0));
	cv::rectangle(mask, roi, cv::Scalar(255), cv::FILLED);

	dv::EventFilterChain filter;

	filter.addFilter(std::make_shared<dv::EventMaskFilter<>>(mask));
	filter.addFilter(std::make_shared<dv::EventPolarityFilter<>>(true));
	filter.addFilter(std::make_shared<dv::noise::BackgroundActivityNoiseFilter<>>(resolution));

	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	const uint32_t eventCount = state.range(0);

	int64_t timestamp = 1000;
	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(timestamp, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	size_t counter  = 0;
	size_t filtered = 0;
	for (auto _ : state) {
		filter.accept(store);
		filtered += filter.generateEvents().size();
		counter  += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

int main(int argc, char **argv) {
	BENCHMARK(bmFastDecayNoiseFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmBackgroundActivityNoiseFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmRefractoryPeriodFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmPolarityFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmRegionFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmMaskFilter)->Range(1'000, 1'000'000);
	BENCHMARK(bmThreeFiltersNoChain)->Range(1'000, 1'000'000);
	BENCHMARK(bmThreeFiltersWithChain)->Range(1'000, 1'000'000);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

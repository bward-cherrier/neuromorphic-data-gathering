#include "../../include/dv-processing/core/core.hpp"

#include <benchmark/benchmark.h>

#include <random>

static void generateEvent(const size_t numOfEvents, dv::EventStore &events) {
	std::default_random_engine rng;
	std::uniform_real_distribution uniform;
	cv::Size resolution(100, 100);

	// Generate uniformly distributed events
	int64_t timestamp = 100'000'000;
	for (size_t i = 0; i < numOfEvents; i++) {
		events.emplace_back(
			timestamp, resolution.width * uniform(rng), resolution.height * uniform(rng), uniform(rng) > 0.5);
	}
}

template<int SHARD_SIZE>
void bmEventStoreSharding(benchmark::State &state) {
	const size_t numOfEvents = state.range(0);

	dv::EventStore events;
	events.setShardCapacity(SHARD_SIZE);

	// Generate an event store with uniformly distributed events
	generateEvent(numOfEvents, events);

	int64_t sliceStart = events.getLowestTime() + static_cast<int64_t>((0.25 * events.duration()).count());
	int64_t sliceEnd   = events.getLowestTime() + static_cast<int64_t>((0.75 * events.duration()).count());

	dv::EventStore copyLocation1;
	dv::EventStore copyLocation2;

	while (state.KeepRunning()) {
		// Slicing
		copyLocation1.add(events.sliceTime(sliceStart, sliceEnd).sliceBack(1000));

		// Random access
		copyLocation2.push_back(events[events.size() * 0.1]);
		copyLocation2.push_back(events[events.size() * 0.2]);
		copyLocation2.push_back(events[events.size() * 0.3]);
		copyLocation2.push_back(events[events.size() * 0.4]);
		copyLocation2.push_back(events[events.size() * 0.5]);
		copyLocation2.push_back(events[events.size() * 0.6]);
		copyLocation2.push_back(events[events.size() * 0.7]);
		copyLocation2.push_back(events[events.size() * 0.8]);
		copyLocation2.push_back(events[events.size() * 0.9]);
	}

	state.SetComplexityN(numOfEvents);
}

int main(int argc, char **argv) {
	static constexpr auto start = 10'000;
	static constexpr auto stop  = 10'000'000;
	::benchmark::internal::RegisterBenchmarkInternal(
		new ::benchmark::internal::FunctionBenchmark("bmEventStoreSharding<1>", bmEventStoreSharding<1>))
		->Range(start, stop)
		->Complexity();
	::benchmark::internal::RegisterBenchmarkInternal(
		new ::benchmark::internal::FunctionBenchmark("bmEventStoreSharding<10>", bmEventStoreSharding<10>))
		->Range(start, stop)
		->Complexity();
	::benchmark::internal::RegisterBenchmarkInternal(
		new ::benchmark::internal::FunctionBenchmark("bmEventStoreSharding<100>", bmEventStoreSharding<100>))
		->Range(start, stop)
		->Complexity();
	::benchmark::internal::RegisterBenchmarkInternal(
		new ::benchmark::internal::FunctionBenchmark("bmEventStoreSharding<1000>", bmEventStoreSharding<1000>))
		->Range(start, stop)
		->Complexity();
	::benchmark::internal::RegisterBenchmarkInternal(
		new ::benchmark::internal::FunctionBenchmark("bmEventStoreSharding<10000>", bmEventStoreSharding<10000>))
		->Range(start, stop)
		->Complexity();

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

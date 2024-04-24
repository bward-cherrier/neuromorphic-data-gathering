#include "../../include/dv-processing/containers/kd_tree.hpp"

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <random>
#include <unordered_set>

using namespace dv::containers::kd_tree;

std::unordered_set<uint64_t> generateRandomButUniqueIndices(const uint64_t numValues, const uint64_t maxIndex) {
	std::unordered_set<uint64_t> expectedIndices;

	static constexpr uint32_t SEED = 0;
	std::default_random_engine generator(SEED);
	std::uniform_int_distribution<uint64_t> distribution(0, maxIndex);

	for (uint8_t i = 0; i < numValues; i++) {
		auto randomIndex = 0;
		do {
			randomIndex = distribution(generator);
		}
		while (expectedIndices.find(randomIndex) != expectedIndices.end());
		expectedIndices.insert(randomIndex);
	}

	return expectedIndices;
}

void bmEventStoreSearch(benchmark::State &state) {
	const uint32_t numSamples = state.range(0);

	dv::EventStore events;

	for (uint64_t i = 0; i < numSamples; i++) {
		events << dv::Event(0, 0, 0, 0);
	}

	static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
	static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

	const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, numSamples - 1);

	for (uint64_t i = 0; i < numSamples; i++) {
		if (expectedIndices.find(i) != expectedIndices.end()) {
			events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
		}
		else {
			events << dv::Event(0, 0, 0, 0);
		}
	}

	dv::Event centre(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
	static constexpr auto RADIUS = 1;

	for (auto _ : state) {
		const auto tree = KDTreeEventStoreAdaptor(events);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		if (indicesAndDistances.size() != EXPECTED_NUM_NEIGHBOURS) {
			throw std::runtime_error("Unexpected numbers of neighbours found");
		}
	}

	state.SetComplexityN(state.range(0));
}

void bmMatrixSearch(benchmark::State &state) {
	static constexpr auto DIMS = 2;
	const uint32_t numSamples  = state.range(0);

	using Tree = KDTreeColMajorXX<int32_t>;

	Tree::Matrix data(DIMS, numSamples);

	static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
	static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

	const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, numSamples - 1);

	for (uint64_t i = 0; i < numSamples; i++) {
		if (expectedIndices.find(i) != expectedIndices.end()) {
			data(0, i) = NEIGHBOURS_VALUE;
			data(1, i) = NEIGHBOURS_VALUE;
		}
		else {
			data(0, i) = 0;
			data(1, i) = 0;
		}
	}

	Eigen::Matrix<int32_t, 2, 1> centre;
	static constexpr auto RADIUS = 1;
	centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

	for (auto _ : state) {
		const auto tree = Tree(data);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		if (indicesAndDistances.size() != EXPECTED_NUM_NEIGHBOURS) {
			throw std::runtime_error("Unexpected numbers of neighbours found");
		}
	}

	state.SetComplexityN(state.range(0));
}

int main(int argc, char **argv) {
	static constexpr auto start = 1E5;
	static constexpr auto stop  = 5E7;
	static constexpr auto step  = (stop - start) / 20.0;

	BENCHMARK(bmMatrixSearch)->DenseRange(start, stop, step)->Complexity(benchmark::oN);
	BENCHMARK(bmEventStoreSearch)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

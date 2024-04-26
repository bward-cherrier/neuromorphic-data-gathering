#include "mean-shift-alternatives.hpp"

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <random>

using namespace dv::cluster::mean_shift;

static constexpr auto MEAN                 = 100.0f;
static constexpr auto STD_DEV              = 15.0f;
static constexpr auto BW                   = 0.5f * STD_DEV;
static constexpr auto CONV                 = 1E-3;
static constexpr auto MAX_ITER             = 10000;
static constexpr auto STARTINGPOINT_OFFSET = 0.25f * STD_DEV;

auto generate_eigen_matrix_data(const uint32_t numSamples) {
	using MeanShift       = MeanShiftRowMajorMatrixX2<float>;
	using Matrix          = MeanShift::Matrix;
	using Vector          = MeanShift::Vector;
	using VectorOfVectors = MeanShift::VectorOfVectors;

	static constexpr uint32_t SEED = 0;
	std::default_random_engine generator(SEED);
	std::normal_distribution<float> distribution(MEAN, STD_DEV);

	auto data = Matrix(numSamples, 2);
	for (uint64_t i = 0; i < numSamples; i++) {
		data(i, 0) = distribution(generator);
		data(i, 1) = distribution(generator);
	}

	VectorOfVectors startingPoints(1, Vector(1, 2));

	startingPoints.back() << MEAN - STARTINGPOINT_OFFSET, MEAN + STARTINGPOINT_OFFSET;

	const auto expected = MeanShift::Vector::Constant(1, 2, MEAN);

	return std::make_tuple(data, startingPoints, expected);
}

auto generate_event_store_data(const uint32_t numSamples) {
	using MeanShift       = MeanShiftEventStoreAdaptor;
	using VectorOfVectors = MeanShift::VectorOfVectors;

	static constexpr uint32_t SEED = 0;
	std::default_random_engine generator(SEED);
	std::normal_distribution<float> distribution(MEAN, STD_DEV);

	dv::EventStore data;
	for (uint64_t i = 0; i < numSamples; i++) {
		data << dv::Event(0, static_cast<int16_t>(std::round(distribution(generator))),
			static_cast<int16_t>(std::round(distribution(generator))), 0);
	}

	VectorOfVectors startingPoints;
	startingPoints.emplace_back(dv::TimedKeyPoint(
		dv::Point2f(MEAN - STARTINGPOINT_OFFSET, MEAN + STARTINGPOINT_OFFSET), 0.0, 0.0, 0.0, 0, 0, 0));

	const auto expected = dv::TimedKeyPoint(dv::Point2f(MEAN, MEAN), 0.0, 0.0, 0.0, 0, 0, 0);

	return std::make_tuple(data, startingPoints, expected);
}

template<kernel::MeanShiftKernel kernel>
void bmEigenMatrix(benchmark::State &state) {
	using MeanShift = MeanShiftRowMajorMatrixX2<float>;

	const uint32_t numSamples = state.range(0);
	const uint32_t numLeaves  = 32000;

	const auto [data, startingPoints, expected] = generate_eigen_matrix_data(numSamples);

	for (auto _ : state) {
		MeanShift meanShift(data, BW, CONV, MAX_ITER, std::move(startingPoints), numLeaves);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel>();

		if (centres.size() != 1) {
			throw std::runtime_error("Centres size is incorrect.");
		}

		if ((centres[0] - expected).norm() > STD_DEV) {
			throw std::runtime_error(fmt::format("Computed centre is incorrect. Expected: [{}, {}], actual: [{}, {}]",
				centres[0](0), centres[0](1), expected(0), expected(1)));
		}
	}

	state.SetComplexityN(state.range(0));
}

template<kernel::MeanShiftKernel kernel>
void bmEventStore(benchmark::State &state) {
	using MeanShift = MeanShiftEventStoreAdaptor;

	const uint32_t numSamples = state.range(0);
	const uint32_t numLeaves  = 32000;

	const auto [data, startingPoints, expected] = generate_event_store_data(numSamples);

	for (auto _ : state) {
		MeanShift meanShift(data, BW, CONV, MAX_ITER, std::move(startingPoints), numLeaves);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel>();

		if (centres.size() != 1) {
			throw std::runtime_error("Centres size is incorrect.");
		}

		if ((centres[0].pt.x() - expected.pt.x()) * (centres[0].pt.x() - expected.pt.x())
				+ (centres[0].pt.y() - expected.pt.y()) * (centres[0].pt.y() - expected.pt.y())
			> STD_DEV * STD_DEV) {
			throw std::runtime_error(fmt::format("Computed centre is incorrect. Expected: [{}, {}], actual: [{}, {}]",
				centres[0].pt.x(), centres[0].pt.y(), expected.pt.x(), expected.pt.y()));
		}
	}

	state.SetComplexityN(state.range(0));
}

void bmLinearSearch(benchmark::State &state) {
	using MeanShift = MeanShiftLinearSearch<float, -1, 2, Eigen::RowMajor>;

	const uint32_t numSamples = state.range(0);

	const auto [data, startingPoints, expected] = generate_eigen_matrix_data(numSamples);

	for (auto _ : state) {
		MeanShift meanShift(data, BW, CONV, MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit();

		if (centres.size() != 1) {
			throw std::runtime_error("Centres size is incorrect.");
		}

		if ((centres[0] - expected).norm() > STD_DEV) {
			throw std::runtime_error(fmt::format("Computed centre is incorrect. Expected: [{}, {}], actual: [{}, {}]",
				centres[0](0), centres[0](1), expected(0), expected(1)));
		}
	}

	state.SetComplexityN(state.range(0));
}

void bmRTree(benchmark::State &state) {
	using MeanShift = MeanShiftRTree<float, -1, 2, Eigen::RowMajor>;

	const uint32_t numSamples = state.range(0);

	const auto [data, startingPoints, expected] = generate_eigen_matrix_data(numSamples);

	for (auto _ : state) {
		MeanShift meanShift(data, BW, CONV, MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit();

		if (centres.size() != 1) {
			throw std::runtime_error("Centres size is incorrect.");
		}

		if ((centres[0] - expected).norm() > STD_DEV) {
			throw std::runtime_error(fmt::format("Computed centre is incorrect. Expected: [{}, {}], actual: [{}, {}]",
				centres[0](0), centres[0](1), expected(0), expected(1)));
		}
	}

	state.SetComplexityN(state.range(0));
}

void bmRTreeBBox(benchmark::State &state) {
	using MeanShift = MeanShiftRTree<float, -1, 2, Eigen::RowMajor>;

	const uint32_t numSamples = state.range(0);

	const auto [data, startingPoints, expected] = generate_eigen_matrix_data(numSamples);

	for (auto _ : state) {
		MeanShift meanShift(data, BW, CONV, MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fitBBox();

		if (centres.size() != 1) {
			throw std::runtime_error("Centres size is incorrect.");
		}

		if ((centres[0] - expected).norm() > STD_DEV) {
			throw std::runtime_error(fmt::format("Computed centre is incorrect. Expected: [{}, {}], actual: [{}, {}]",
				centres[0](0), centres[0](1), expected(0), expected(1)));
		}
	}

	state.SetComplexityN(state.range(0));
}

int main(int argc, char **argv) {
	static constexpr auto start = 1E5;
	static constexpr auto stop  = 3E6;
	static constexpr auto step  = (stop - start) / 30.0;

	BENCHMARK_TEMPLATE(bmEigenMatrix, kernel::Gaussian)->DenseRange(start, stop, step)->Complexity(benchmark::oN);
	BENCHMARK_TEMPLATE(bmEigenMatrix, kernel::Epanechnikov)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	BENCHMARK_TEMPLATE(bmEventStore, kernel::Gaussian)->DenseRange(start, stop, step)->Complexity(benchmark::oN);
	BENCHMARK_TEMPLATE(bmEventStore, kernel::Epanechnikov)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	BENCHMARK(bmLinearSearch)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	BENCHMARK(bmRTree)->DenseRange(start, stop, step)->Complexity(benchmark::oN);
	BENCHMARK(bmRTreeBBox)->DenseRange(start, stop, step)->Complexity(benchmark::oN);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

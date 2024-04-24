#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/kinematics/motion_compensator.hpp"
#include "../../include/dv-processing/visualization/event_visualizer.hpp"

#include <benchmark/benchmark.h>

#include <random>

static const size_t eventCount     = 100000;
static const size_t iterationCount = 1500;
static const cv::Size resolution(640, 480);

template<class Accumulator>
void bmFrameRate(benchmark::State &state) {
	Accumulator accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}

	int64_t counter = 0;
	dv::Frame frame(0, cv::Mat(resolution, CV_8UC1));
	for (auto _ : state) {
		accumulator.accept(store);
		frame = accumulator.generateFrame();
		counter++;
	}
	state.SetItemsProcessed(counter);
	state.SetLabel("frames");
}

template<class Accumulator>
void bmEventRate(benchmark::State &state) {
	Accumulator accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	size_t counter = 0;
	dv::Frame frame(0, cv::Mat(resolution, CV_8UC1));
	for (auto _ : state) {
		accumulator.accept(store);
		frame   = accumulator.generateFrame();
		counter += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmMotionCompensatorEventRate(benchmark::State &state) {
	dv::kinematics::MotionCompensator accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(1000, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	accumulator.accept(dv::kinematics::Transformationf(500, Eigen::Matrix4f::Identity()));
	accumulator.accept(dv::kinematics::Transformationf(1000, Eigen::Matrix4f::Identity()));
	accumulator.accept(dv::kinematics::Transformationf(1500, Eigen::Matrix4f::Identity()));

	accumulator.accept(dv::measurements::Depth(1000, 3.f));

	size_t counter = 0;
	for (auto _ : state) {
		accumulator.accept(store);
		accumulator.generateFrame();
		counter += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

static void bmMotionCompensatorFrameRate(benchmark::State &state) {
	dv::kinematics::MotionCompensator accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(1000, distribution(generator) * resolution.width,
			distribution(generator) * resolution.height, distribution(generator) > 0.5);
	}

	accumulator.accept(dv::kinematics::Transformationf(500, Eigen::Matrix4f::Identity()));
	accumulator.accept(dv::kinematics::Transformationf(1000, Eigen::Matrix4f::Identity()));
	accumulator.accept(dv::kinematics::Transformationf(1500, Eigen::Matrix4f::Identity()));

	accumulator.accept(dv::measurements::Depth(1000, 3.f));

	int64_t counter = 0;
	for (auto _ : state) {
		accumulator.accept(store);
		accumulator.generateFrame();
		counter++;
	}
	state.SetItemsProcessed(counter);
	state.SetLabel("frames");
}

void bmEventVisualizerFrameRate(benchmark::State &state) {
	dv::visualization::EventVisualizer accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}

	int64_t counter = 0;
	for (auto _ : state) {
		cv::Mat image = accumulator.generateImage(store);
		counter++;
	}
	state.SetItemsProcessed(counter);
	state.SetLabel("frames");
}

void bmEventVisualizerEventRate(benchmark::State &state) {
	dv::visualization::EventVisualizer accumulator(resolution);
	dv::EventStore store;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	for (size_t i = 0; i < eventCount; i++) {
		store.emplace_back(0, distribution(generator) * resolution.width, distribution(generator) * resolution.height,
			distribution(generator) > 0.5);
	}
	size_t counter = 0;
	for (auto _ : state) {
		cv::Mat image = accumulator.generateImage(store);
		counter       += store.size();
	}
	state.SetItemsProcessed(static_cast<int64_t>(counter));
	state.SetLabel("events");
}

int main(int argc, char **argv) {
	BENCHMARK_TEMPLATE(bmFrameRate, dv::EdgeMapAccumulator);
	BENCHMARK_TEMPLATE(bmEventRate, dv::EdgeMapAccumulator);
	BENCHMARK_TEMPLATE(bmFrameRate, dv::Accumulator);
	BENCHMARK_TEMPLATE(bmEventRate, dv::Accumulator);
	BENCHMARK_TEMPLATE(bmFrameRate, dv::TimeSurface);
	BENCHMARK_TEMPLATE(bmEventRate, dv::TimeSurface);
	BENCHMARK_TEMPLATE(bmFrameRate, dv::SpeedInvariantTimeSurface);
	BENCHMARK_TEMPLATE(bmEventRate, dv::SpeedInvariantTimeSurface);

	// Motion compensator and event visualizer needs different implementation for benchmark
	BENCHMARK(bmMotionCompensatorFrameRate);
	BENCHMARK(bmMotionCompensatorEventRate);
	BENCHMARK(bmEventVisualizerFrameRate);
	BENCHMARK(bmEventVisualizerEventRate);

	::benchmark::Initialize(&argc, argv);
	if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
		return 1;
	}
	::benchmark::RunSpecifiedBenchmarks();

	return EXIT_SUCCESS;
}

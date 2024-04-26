#include "../../include/dv-processing/cluster/mean_shift.hpp"

#include "boost/ut.hpp"

#include <Eigen/Dense>

#include <random>

using namespace boost::ut;
using namespace dv::cluster::mean_shift;

struct TestParamsMatrix {
	static constexpr auto NUM_SAMPLES          = 10000;
	static constexpr auto STD_DEV              = 0.3f;
	static constexpr auto VAR_TOL              = 0.1f * STD_DEV * STD_DEV;
	static constexpr auto CONV                 = 1E-5;
	static constexpr auto MAX_ITER             = 10000;
	static constexpr auto NUM_STARTING_POINTS  = 10;
	static constexpr auto STARTINGPOINT_OFFSET = 0.25f * STD_DEV;
	static constexpr auto BW                   = 1.0f * STD_DEV;
	static constexpr float MEAN1               = 0.0f;
	static constexpr float MEAN2               = 100.0f;
};

struct TestParamsEventStore {
	static constexpr auto NUM_SAMPLES          = 10000;
	static constexpr auto STD_DEV              = 3.0f;
	static constexpr auto VAR_TOL              = 0.05f * STD_DEV * STD_DEV;
	static constexpr auto CONV                 = 1E-5;
	static constexpr auto MAX_ITER             = 10000;
	static constexpr auto NUM_STARTING_POINTS  = 10;
	static constexpr auto STARTINGPOINT_OFFSET = 0.25f * STD_DEV;
	static constexpr auto BW                   = 3.0f * STD_DEV;
	static constexpr float MEAN1               = 0.0f;
	static constexpr float MEAN2               = 100.0f;
};

auto rng(const float mean, const float stdDev) {
	static constexpr uint32_t SEED = 0;
	std::default_random_engine generator(SEED);
	std::normal_distribution<float> distribution(mean, stdDev);
	return std::bind(distribution, generator);
}

void testMatrix() {
	using params = TestParamsMatrix;
	auto rng1    = rng(params::MEAN1, params::STD_DEV);
	auto rng2    = rng(params::MEAN2, params::STD_DEV);

	"EigenMatrix_1d_dynamic_size_row_major"_test = [&] {
		auto data = MeanShiftRowMajorMatrixXX<float>::Matrix(params::NUM_SAMPLES, 1);
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i) = rng1();
		}

		MeanShiftRowMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
		expect(std::abs(variances[0] - params::STD_DEV * params::STD_DEV) < params::VAR_TOL);
	};

	"EigenMatrix_1d_dynamic_size_column_major"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(1, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i) = rng1();
		}

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected = MeanShiftColMajorMatrixXX<float>::Vector::Constant(1, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
		expect(std::abs(variances[0] - params::STD_DEV * params::STD_DEV) < params::VAR_TOL);
	};

	"EigenMatrix_1d_static_size_row_major"_test = [&] {
		MeanShiftRowMajorMatrixX1<float, params::NUM_SAMPLES>::Matrix data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i) = rng1();
		}

		MeanShiftRowMajorMatrixX1<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftRowMajorMatrixX1<float, params::NUM_SAMPLES>::Vector::Constant(1, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
		expect(std::abs(variances[0] - params::STD_DEV * params::STD_DEV) < params::VAR_TOL);
	};

	"EigenMatrix_1d_static_size_column_major"_test = [&] {
		MeanShiftColMajorMatrix1X<float, params::NUM_SAMPLES>::Matrix data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i) = rng1();
		}

		MeanShiftColMajorMatrix1X<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftColMajorMatrix1X<float, params::NUM_SAMPLES>::Vector::Constant(1, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
		expect(std::abs(variances[0] - params::STD_DEV * params::STD_DEV) < params::VAR_TOL);
	};

	"EigenMatrix_2d_dynamic_size_row_major"_test = [&] {
		auto data = MeanShiftRowMajorMatrixXX<float>::Matrix(params::NUM_SAMPLES, 2);
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		MeanShiftRowMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EigenMatrix_2d_dynamic_size_column_major"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(2, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EigenMatrix_2d_static_size_row_major"_test = [&] {
		auto data = MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES>::Matrix();
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES>::Vector::Constant(1, 2, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EigenMatrix_2d_static_size_column_major"_test = [&] {
		auto data = MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES>::Matrix();
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES>::Vector::Constant(2, 1, params::MEAN1);

		expect(centres.size() == 1);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect((centres[0] - expected).norm() < params::STD_DEV / 3.0f);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EigenMatrix_2d_dynamic_size_row_major_2clusters_customStartingPoints"_test = [&] {
		auto data = MeanShiftRowMajorMatrixXX<float>::Matrix(params::NUM_SAMPLES, 2);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng2();
			data(i, 1) = rng2();
		}

		MeanShiftRowMajorMatrixXX<float>::VectorOfVectors startingPoints(
			2, MeanShiftRowMajorMatrixXX<float>::Vector(1, 2));

		startingPoints[0] << params::MEAN1 + params::STARTINGPOINT_OFFSET, params::MEAN1 - params::STARTINGPOINT_OFFSET;
		startingPoints[1] << params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET;

		MeanShiftRowMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN1);
		const auto expected2 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect(centres[1].rows() == 1);
		expect(centres[1].cols() == 2);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_dynamic_size_row_major_2clusters_customStartingPoints_fromRange"_test = [&] {
		auto data = MeanShiftRowMajorMatrixXX<float>::Matrix(params::NUM_SAMPLES, 2);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng2();
			data(i, 1) = rng2();
		}

		MeanShiftRowMajorMatrixXX<float>::VectorOfVectors startingPoints
			= MeanShiftRowMajorMatrixXX<float>::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN1 - params::STARTINGPOINT_OFFSET, params::MEAN1 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN1 - params::STARTINGPOINT_OFFSET,
							 params::MEAN1 + params::STARTINGPOINT_OFFSET)});

		MeanShiftRowMajorMatrixXX<float>::VectorOfVectors startingPoints2
			= MeanShiftRowMajorMatrixXX<float>::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN2 - params::STARTINGPOINT_OFFSET,
							 params::MEAN2 + params::STARTINGPOINT_OFFSET)});

		startingPoints.insert(startingPoints.end(), startingPoints2.begin(), startingPoints2.end());

		MeanShiftRowMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Gaussian>();

		const auto expected1 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN1);
		const auto expected2 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect(centres[1].rows() == 1);
		expect(centres[1].cols() == 2);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_dynamic_size_row_major_2clusters_customStartingPoints_fromData"_test = [&] {
		auto data = MeanShiftRowMajorMatrixXX<float>::Matrix(params::NUM_SAMPLES, 2);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng2();
			data(i, 1) = rng2();
		}

		MeanShiftRowMajorMatrixXX<float>::VectorOfVectors startingPoints
			= MeanShiftRowMajorMatrixXX<float>::generateStartingPointsFromData(10, data);

		MeanShiftRowMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Gaussian>();

		const auto expected1 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN1);
		const auto expected2 = MeanShiftRowMajorMatrixXX<float>::Vector::Constant(1, 2, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect(centres[1].rows() == 1);
		expect(centres[1].cols() == 2);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_dynamic_size_column_major_2clusters_customStartingPoints"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(2, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng2();
			data(1, i) = rng2();
		}

		MeanShiftColMajorMatrixXX<float>::VectorOfVectors startingPoints(
			2, MeanShiftColMajorMatrixXX<float>::Vector(2, 1));

		startingPoints[0] << params::MEAN1 + params::STARTINGPOINT_OFFSET, params::MEAN1 - params::STARTINGPOINT_OFFSET;
		startingPoints[1] << params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET;

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN1);
		const auto expected2 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect(centres[1].rows() == 2);
		expect(centres[1].cols() == 1);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_dynamic_size_column_major_2clusters_customStartingPoints_fromRange"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(2, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng2();
			data(1, i) = rng2();
		}

		MeanShiftColMajorMatrixXX<float>::VectorOfVectors startingPoints
			= MeanShiftColMajorMatrixXX<float>::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN1 - params::STARTINGPOINT_OFFSET, params::MEAN1 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN1 - params::STARTINGPOINT_OFFSET,
							 params::MEAN1 + params::STARTINGPOINT_OFFSET)});

		MeanShiftColMajorMatrixXX<float>::VectorOfVectors startingPoints2
			= MeanShiftColMajorMatrixXX<float>::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN2 - params::STARTINGPOINT_OFFSET,
							 params::MEAN2 + params::STARTINGPOINT_OFFSET)});

		startingPoints.insert(startingPoints.end(), startingPoints2.begin(), startingPoints2.end());

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Gaussian>();

		const auto expected1 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN1);
		const auto expected2 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect(centres[1].rows() == 2);
		expect(centres[1].cols() == 1);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_dynamic_size_column_major_2clusters_customStartingPoints_fromData"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(2, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng2();
			data(1, i) = rng2();
		}

		MeanShiftColMajorMatrixXX<float>::VectorOfVectors startingPoints
			= MeanShiftColMajorMatrixXX<float>::generateStartingPointsFromData(10, data);

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN1);
		const auto expected2 = MeanShiftColMajorMatrixXX<float>::Vector::Constant(2, 1, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect(centres[1].rows() == 2);
		expect(centres[1].cols() == 1);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_static_size_row_major_2clusters_customStartingPoints"_test = [&] {
		auto data = MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES>::Matrix();
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(i, 0) = rng1();
			data(i, 1) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(i, 0) = rng2();
			data(i, 1) = rng2();
		}

		MeanShiftRowMajorMatrixX2<float>::VectorOfVectors startingPoints(
			2, MeanShiftRowMajorMatrixX2<float>::Vector(2, 1));

		startingPoints[0] << params::MEAN1 + params::STARTINGPOINT_OFFSET, params::MEAN1 - params::STARTINGPOINT_OFFSET;
		startingPoints[1] << params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET;

		MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1
			= MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES>::Vector::Constant(1, 2, params::MEAN1);
		const auto expected2
			= MeanShiftRowMajorMatrixX2<float, params::NUM_SAMPLES>::Vector::Constant(1, 2, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 1);
		expect(centres[0].cols() == 2);
		expect(centres[1].rows() == 1);
		expect(centres[1].cols() == 2);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));

		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_2d_static_size_column_major_2clusters_customStartingPoints"_test = [&] {
		auto data = MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES>::Matrix();
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2 - 1; i++) {
			data(0, i) = rng1();
			data(1, i) = rng1();
		}

		for (uint64_t i = params::NUM_SAMPLES / 2; i < params::NUM_SAMPLES; i++) {
			data(0, i) = rng2();
			data(1, i) = rng2();
		}

		MeanShiftColMajorMatrix2X<float>::VectorOfVectors startingPoints(
			2, MeanShiftColMajorMatrix2X<float>::Vector(2, 1));

		startingPoints[0] << params::MEAN1 + params::STARTINGPOINT_OFFSET, params::MEAN1 - params::STARTINGPOINT_OFFSET;
		startingPoints[1] << params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET;

		MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1
			= MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES>::Vector::Constant(2, 1, params::MEAN1);
		const auto expected2
			= MeanShiftColMajorMatrix2X<float, params::NUM_SAMPLES>::Vector::Constant(2, 1, params::MEAN2);

		expect(centres.size() == 2);
		expect(centres[0].rows() == 2);
		expect(centres[0].cols() == 1);
		expect(centres[1].rows() == 2);
		expect(centres[1].cols() == 1);
		expect((((centres[0] - expected1).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected2).norm() < params::STD_DEV / 3.0f))
			   || (((centres[0] - expected2).norm() < params::STD_DEV / 3.0f)
				   && ((centres[1] - expected1).norm() < params::STD_DEV / 3.0f)));
		expect(counts[0] == params::NUM_SAMPLES / 2);
		expect(counts[1] == params::NUM_SAMPLES / 2);
	};

	"EigenMatrix_noCluster"_test = [&] {
		auto data = Eigen::MatrixXf(1, params::NUM_SAMPLES);
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data(i) = rng1();
		}

		MeanShiftEigenMatrixAdaptor<float>::VectorOfVectors startingPoints(
			1, MeanShiftEigenMatrixAdaptor<float>::Vector(1, 1));

		startingPoints[0] << 100;

		MeanShiftEigenMatrixAdaptor<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		expect(centres.size() == 0);
	};

	"EigenMatrix_noData"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(1, 0);

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		expect(centres.size() == 0);
	};

	"EigenMatrix_noDims"_test = [&] {
		auto data = MeanShiftColMajorMatrixXX<float>::Matrix(0, params::NUM_SAMPLES);

		MeanShiftColMajorMatrixXX<float> meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		expect(centres.size() == 0);
	};
}

float squaredDistance(const dv::TimedKeyPoint &k1, const dv::TimedKeyPoint &k2) {
	return (k1.pt.x() - k2.pt.x()) * (k1.pt.x() - k2.pt.x()) + (k1.pt.y() - k2.pt.y()) * (k1.pt.y() - k2.pt.y());
}

void testEventStore() {
	using params = TestParamsEventStore;
	auto rng1    = rng(params::MEAN1, params::STD_DEV);
	auto rng2    = rng(params::MEAN2, params::STD_DEV);

	"EventStore_1cluster_randomStaringPoints"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data << dv::Event(
				0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), false);
		}

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(centres.size() == 1);
		expect(squaredDistance(expected, centres[0]) < params::STD_DEV * params::STD_DEV);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EventStore_1cluster_customStartingPoints"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints;
		startingPoints.emplace_back(dv::TimedKeyPoint(
			dv::Point2f(params::MEAN1 - params::STARTINGPOINT_OFFSET, params::MEAN1 + params::STARTINGPOINT_OFFSET),
			0.0, 0.0, 0.0, 0, 0, 0));

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(centres.size() == 1);
		expect(squaredDistance(expected, centres[0]) < params::STD_DEV * params::STD_DEV);
		expect(counts[0] == params::NUM_SAMPLES);
	};

	"EventStore_2clusters_randomStaringPoints"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng2())), static_cast<int16_t>(std::round(rng2())), 0);
		}

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Gaussian>();

		const auto expected1
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);
		const auto expected2
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN2, params::MEAN2), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(eq(centres.size(), 2));
		expect(((squaredDistance(centres[0], expected1) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected2) < params::STD_DEV * params::STD_DEV))
			   || ((squaredDistance(centres[0], expected2) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected1) < params::STD_DEV * params::STD_DEV)));
		expect(counts[0] == params::NUM_SAMPLES / 2 && counts[1] == params::NUM_SAMPLES / 2);
	};

	"EventStore_2clusters_customStartingPoints"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng2())), static_cast<int16_t>(std::round(rng2())), 0);
		}

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints;
		startingPoints.emplace_back(dv::TimedKeyPoint(
			dv::Point2f(params::MEAN1 - params::STARTINGPOINT_OFFSET, params::MEAN1 + params::STARTINGPOINT_OFFSET),
			0.0, 0.0, 0.0, 0, 0, 0));
		startingPoints.emplace_back(dv::TimedKeyPoint(
			dv::Point2f(params::MEAN2 + params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET),
			0.0, 0.0, 0.0, 0, 0, 0));

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);
		const auto expected2
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN2, params::MEAN2), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(centres.size() == 2);
		expect(((squaredDistance(centres[0], expected1) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected2) < params::STD_DEV * params::STD_DEV))
			   || ((squaredDistance(centres[0], expected2) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected1) < params::STD_DEV * params::STD_DEV)));
		expect(counts[0] == params::NUM_SAMPLES / 2 && counts[1] == params::NUM_SAMPLES / 2);
	};

	"EventStore_2clusters_customStartingPoints_fromRange"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng2())), static_cast<int16_t>(std::round(rng2())), 0);
		}

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints
			= MeanShiftEventStoreAdaptor::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN1 - params::STARTINGPOINT_OFFSET, params::MEAN1 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN1 - params::STARTINGPOINT_OFFSET,
							 params::MEAN1 + params::STARTINGPOINT_OFFSET)});

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints2
			= MeanShiftEventStoreAdaptor::generateStartingPointsFromRange(
				100, {std::make_pair(
						  params::MEAN2 - params::STARTINGPOINT_OFFSET, params::MEAN2 + params::STARTINGPOINT_OFFSET),
						 std::make_pair(params::MEAN2 - params::STARTINGPOINT_OFFSET,
							 params::MEAN2 + params::STARTINGPOINT_OFFSET)});

		startingPoints.insert(startingPoints.end(), startingPoints2.begin(), startingPoints2.end());

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		const auto expected1
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);
		const auto expected2
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN2, params::MEAN2), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(centres.size() == 2);
		expect(((squaredDistance(centres[0], expected1) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected2) < params::STD_DEV * params::STD_DEV))
			   || ((squaredDistance(centres[0], expected2) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected1) < params::STD_DEV * params::STD_DEV)));
		expect(counts[0] == params::NUM_SAMPLES / 2 && counts[1] == params::NUM_SAMPLES / 2);
	};

	"EventStore_2clusters_customStartingPoints_fromData"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		for (uint64_t i = 0; i < params::NUM_SAMPLES / 2; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng2())), static_cast<int16_t>(std::round(rng2())), 0);
		}

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints
			= MeanShiftEventStoreAdaptor::generateStartingPointsFromData(100, data);

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Gaussian>();

		const auto expected1
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN1, params::MEAN1), 0.0f, 0.0f, 0.0f, 0, 0, 0);
		const auto expected2
			= MeanShiftEventStoreAdaptor::Vector(dv::Point2f(params::MEAN2, params::MEAN2), 0.0f, 0.0f, 0.0f, 0, 0, 0);

		expect(centres.size() == 2);
		expect(((squaredDistance(centres[0], expected1) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected2) < params::STD_DEV * params::STD_DEV))
			   || ((squaredDistance(centres[0], expected2) < params::STD_DEV * params::STD_DEV)
				   && (squaredDistance(centres[1], expected1) < params::STD_DEV * params::STD_DEV)));
		expect(counts[0] == params::NUM_SAMPLES / 2 && counts[1] == params::NUM_SAMPLES / 2);
	};

	"EventStore_noCluster"_test = [&] {
		dv::EventStore data;
		for (uint64_t i = 0; i < params::NUM_SAMPLES; i++) {
			data << dv::Event(0, static_cast<int16_t>(std::round(rng1())), static_cast<int16_t>(std::round(rng1())), 0);
		}

		MeanShiftEventStoreAdaptor::VectorOfVectors startingPoints;
		startingPoints.emplace_back(dv::TimedKeyPoint(dv::Point2f(1000.0, 1000.0), 0.0, 0.0, 0.0, 0, 0, 0));

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, std::move(startingPoints));
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		expect(centres.size() == 0);
	};

	"EventStore_noData"_test = [&] {
		dv::EventStore data;

		MeanShiftEventStoreAdaptor meanShift(
			data, params::BW, params::CONV, params::MAX_ITER, params::NUM_STARTING_POINTS);
		const auto [centres, labels, counts, variances] = meanShift.fit<kernel::Epanechnikov>();

		expect(centres.size() == 0);
	};
}

int main() {
	testMatrix();
	testEventStore();

	return EXIT_SUCCESS;
}

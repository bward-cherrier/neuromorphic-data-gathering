#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/core/utils.hpp"
#include "../../include/dv-processing/imgproc/imgproc.hpp"

#include "../utilities/CsvEventReader.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

#include <chrono>

using namespace boost::ut;

template<typename T>
void testOCVMat() {
	static constexpr auto NUM_VALUES = 128;
	static constexpr auto MIN_VALUE  = std::numeric_limits<T>::min();
	static constexpr auto MAX_VALUE  = std::numeric_limits<T>::max();

	dv::TimeSurface ts(NUM_VALUES, 1);

	const int64_t timestampBegin = dv::now();
	auto timestamp               = timestampBegin;

	for (uint64_t row = 0; row < ts.rows(); row++) {
		ts(row, 0) = timestamp++;
	}

	const auto [mat, offset] = ts.getOCVMat<T>();

	for (int64_t row = 0; row < ts.rows(); row++) {
		const auto expected = MAX_VALUE - (NUM_VALUES - 1) + row;
		const auto actual   = mat.template at<T>(row, 0);
		expect(eq(expected, actual));

		const auto expectedOffsetCompensated = timestampBegin + row;
		const auto actualOffsetCompensated   = offset + actual;
		expect(eq(expectedOffsetCompensated, actualOffsetCompensated));
	}
}

template<typename T>
void testOCVMatTimeStampRangeExceedsTargetNumericRange() {
	dv::TimeSurface ts(2, 1);

	const int64_t timestampBegin = dv::now();
	const int64_t timestampEnd
		= timestampBegin
		+ (static_cast<int64_t>(std::numeric_limits<T>::max()) - static_cast<int64_t>(std::numeric_limits<T>::min()))
		+ 1;

	ts(0, 0) = timestampBegin;
	ts(1, 0) = timestampEnd;

	const auto [mat, offset] = ts.getOCVMat<T>();

	const auto actualBegin = mat.template at<T>(0, 0);
	const auto actualEnd   = mat.template at<T>(1, 0);
	expect(eq(offset + actualBegin, timestampBegin + 1));
	expect(eq(offset + actualEnd, timestampEnd));
	expect(eq(actualBegin, std::numeric_limits<T>::min()));
	expect(eq(actualEnd, std::numeric_limits<T>::max()));
}

template<typename T>
void testOCVMatScaled() {
	static constexpr auto NUM_VALUES = 128;
	static constexpr auto MIN_VALUE  = std::numeric_limits<T>::min();
	static constexpr auto MAX_VALUE  = std::numeric_limits<T>::max();

	dv::TimeSurface ts(NUM_VALUES, 1);

	const int64_t timestampMin = dv::now();
	const auto timestampMax    = timestampMin + NUM_VALUES - 1;

	auto timestamp = timestampMin;
	for (uint64_t row = 0; row < ts.rows(); row++) {
		ts(row, 0) = timestamp++;
	}

	const auto mat = ts.getOCVMatScaled<T>();

	const auto step = (static_cast<double>(MAX_VALUE) - static_cast<double>(MIN_VALUE))
					/ static_cast<double>(timestampMax - timestampMin);

	for (int64_t row = 0; row < ts.rows(); row++) {
		const auto expected = static_cast<T>(static_cast<double>(MIN_VALUE) + (static_cast<double>(row) * step));
		const auto actual   = mat.template at<T>(row, 0);
		expect(eq(expected, actual));
	}

	static constexpr auto NUM_VALUES_2 = NUM_VALUES / 2;
	static constexpr auto LOOKBACK     = NUM_VALUES_2;
	const auto mat2                    = ts.getOCVMatScaled<T>(LOOKBACK);
	const auto step2
		= (static_cast<double>(MAX_VALUE) - static_cast<double>(MIN_VALUE)) / static_cast<double>(LOOKBACK);

	for (int64_t row = 0; row < ts.rows(); row++) {
		const auto expected
			= row >= NUM_VALUES_2
				? static_cast<T>(static_cast<double>(MIN_VALUE) + (static_cast<double>(row - NUM_VALUES_2 + 1) * step2))
				: MIN_VALUE;
		const auto actual = mat2.template at<T>(row, 0);
		expect(eq(expected, actual));
	}
}

int main() {
	"TimeSurface_OpenCV_Mat_conversion_int8_t"_test = [] {
		testOCVMat<int8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_uint8_t"_test = [] {
		testOCVMat<uint8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_int16_t"_test = [] {
		testOCVMat<int16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_uint16_t"_test = [] {
		testOCVMat<uint16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_int32_t"_test = [] {
		testOCVMat<int32_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_timestamp_range_exceeds_target_numeric_range_int8_t"_test = [] {
		testOCVMatTimeStampRangeExceedsTargetNumericRange<int8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_timestamp_range_exceeds_target_numeric_range_uint8_t"_test = [] {
		testOCVMatTimeStampRangeExceedsTargetNumericRange<uint8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_timestamp_range_exceeds_target_numeric_range_int16_t"_test = [] {
		testOCVMatTimeStampRangeExceedsTargetNumericRange<int16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_timestamp_range_exceeds_target_numeric_range_uint16_t"_test = [] {
		testOCVMatTimeStampRangeExceedsTargetNumericRange<uint16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_timestamp_range_exceeds_target_numeric_range_int32_t"_test = [] {
		testOCVMatTimeStampRangeExceedsTargetNumericRange<int32_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_scaled_int8_t"_test = [] {
		testOCVMatScaled<int8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_scaled_uint8_t"_test = [] {
		testOCVMatScaled<uint8_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_scaled_int16_t"_test = [] {
		testOCVMatScaled<int16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_scaled_uint16_t"_test = [] {
		testOCVMatScaled<uint16_t>();
	};

	"TimeSurface_OpenCV_Mat_conversion_scaled_int32_t"_test = [] {
		testOCVMatScaled<int32_t>();
	};

	"TimeSurface_output_stream_operator"_test = [] {
		static constexpr auto NUM_VALUES = 128;

		dv::TimeSurface ts(NUM_VALUES, 1);

		const int64_t timestampBegin = dv::now();
		auto timestamp               = timestampBegin;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			ts << dv::Event(timestamp, 0, row, false);
			expect(eq(timestamp, ts(row, 0)));
			timestamp++;
		}

		dv::EventStore events;
		timestamp = timestampBegin;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			events << dv::Event(timestamp++, 0, row, false);
		}

		ts << events;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			expect(eq(timestampBegin + row, ts(row, 0)));
		}
	};

	"TimeSurface_input_stream_operator"_test = [] {
		dv::TimeSurface ts(256, 1);

		const int64_t timestampBegin = dv::now();
		auto timestamp               = timestampBegin;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			ts << dv::Event(timestamp, 0, row, false);
			expect(eq(timestamp, ts(row, 0)));
			timestamp++;
		}

		dv::Frame frame;
		ts >> frame;

		for (int64_t row = 0; row < ts.rows(); row++) {
			const auto expected = static_cast<uint8_t>(row);
			const auto actual   = frame.image.at<uint8_t>(row, 0);
			expect(eq(expected, actual));
		}
	};

	"TimeSurface_accept"_test = [] {
		static constexpr auto NUM_VALUES = 128;

		dv::TimeSurface ts(NUM_VALUES, 1);

		const int64_t timestampBegin = dv::now();
		auto timestamp               = timestampBegin;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			ts.accept(dv::Event(timestamp, 0, row, false));
			expect(eq(timestamp, ts(row, 0)));
			timestamp++;
		}

		dv::EventStore events;
		timestamp = timestampBegin;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			events << dv::Event(timestamp++, 0, row, false);
		}

		ts.accept(events);

		for (uint64_t row = 0; row < ts.rows(); row++) {
			expect(eq(timestampBegin + row, ts(row, 0)));
		}
	};

	"TimeSurface_range_check"_test = [] {
		dv::TimeSurface ts(0, 0);

		expect(throws([&] {
			int64_t &val = ts.at(1, 1);
		}));

		expect(throws([&] {
			const int64_t &val = ts.at(1, 1);
		}));
	};

	"TimeSurface_arithmetic"_test = [] {
		static constexpr auto NUM_VALUES = 128;

		dv::TimeSurface ts(NUM_VALUES, 1);

		for (uint64_t row = 0; row < ts.rows(); row++) {
			ts << dv::Event(0, 0, row, false);
		}

		ts += 1;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			expect(eq(ts(row, 0), 1));
		}

		ts -= 1;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			expect(eq(ts(row, 0), 0));
		}

		ts -= 1;

		for (uint64_t row = 0; row < ts.rows(); row++) {
			expect(eq(ts(row, 0), 0));
		}
	};

	"TimeSurface_block"_test = [] {
		dv::TimeSurface ts{5, 5};

		int64_t t = 0;
		for (uint8_t row = 0; row < ts.rows(); row++) {
			for (uint8_t col = 0; col < ts.cols(); col++) {
				ts << dv::Event(t++, col, row, false);
			}
		}

		const auto cBlock = ts.block(1, 0, 2, 1);

		expect(eq(cBlock.rows(), 2));
		expect(eq(cBlock.cols(), 1));

		expect(eq(cBlock(0, 0), 5));
		expect(eq(cBlock(1, 0), 10));

		auto block = ts.block(0, 1, 1, 2);

		expect(eq(block.rows(), 1));
		expect(eq(block.cols(), 2));

		expect(eq(block(0, 0), 1));
		expect(eq(block(0, 1), 2));
	};

	"SpeedInvariantTimeSurface"_test = [&] {
		namespace fs = std::filesystem;

		dv::SpeedInvariantTimeSurface ts{cv::Size(10, 10)};

		ts.block(0, 0, 10, 10) = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(10, 10, 5);

		ts << dv::Event(1, 2, 2, false);

		Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> expected
			= Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(10, 10, 5);
		expected(2, 2) = (8 + 1) * (8 + 1);

		expect(ts.block(0, 0, 10, 10) == expected);

		ts.block(0, 0, 10, 10) = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(10, 10, 5);

		ts(2, 2) = 4;

		ts << dv::Event(1, 2, 2, false);

		expected                   = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(10, 10, 5);
		expected.block(0, 0, 7, 7) -= Eigen::Matrix<uint8_t, 7, 7>::Ones();
		expected(2, 2)             = (8 + 1) * (8 + 1);

		expect(ts.block(0, 0, 10, 10) == expected);
	};

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/imgproc/imgproc.hpp"

#include "boost/ut.hpp"

#include <chrono>

using namespace boost::ut;

template<typename T>
void testL1DistanceOCV() {
	const auto m1 = cv::Mat(5, 5, cv::DataType<T>::type, cv::Scalar::all(0));
	const auto m2 = cv::Mat(5, 5, cv::DataType<T>::type, cv::Scalar::all(1));

	expect(eq(dv::imgproc::L1Distance(m1, m1), 0));
	expect(eq(dv::imgproc::L1Distance(m1, m2), 25));
	expect(eq(dv::imgproc::L1Distance(m1(cv::Rect(1, 1, 4, 4)), m1(cv::Rect(0, 0, 4, 4))), 0));
	expect(eq(dv::imgproc::L1Distance(m1(cv::Rect(1, 1, 4, 4)), m2(cv::Rect(0, 0, 4, 4))), 16));

	const auto m3 = cv::Mat(0, 0, cv::DataType<T>::type);
	const auto m4 = cv::Mat(0, 0, cv::DataType<T>::type);

	expect(eq(dv::imgproc::L1Distance(m3, m3), 0));
	expect(eq(dv::imgproc::L1Distance(m3, m4), 0));
	expect(eq(dv::imgproc::L1Distance(m1(cv::Rect(0, 0, 0, 0)), m1(cv::Rect(0, 0, 0, 0))), 0));
	expect(eq(dv::imgproc::L1Distance(m1(cv::Rect(0, 0, 0, 0)), m2(cv::Rect(0, 0, 0, 0))), 0));
}

template<typename T>
void testL1DistanceEigen() {
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m1
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(5, 5);
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m2
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(5, 5);

	expect(eq(dv::imgproc::L1Distance(m1, m1), 0));
	expect(eq(dv::imgproc::L1Distance(m1, m2), 25));
	expect(eq(dv::imgproc::L1Distance(m1.block(0, 0, 4, 4), m1.block(0, 0, 4, 4)), 0));
	expect(eq(dv::imgproc::L1Distance(m1.block(1, 1, 4, 4), m2.block(0, 0, 4, 4)), 16));

	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m3
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0);
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m4
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(0, 0);

	expect(eq(dv::imgproc::L1Distance(m3, m3), 0));
	expect(eq(dv::imgproc::L1Distance(m3, m4), 0));
	expect(eq(dv::imgproc::L1Distance(m3.block(0, 0, 0, 0), m3.block(0, 0, 0, 0)), 0));
	expect(eq(dv::imgproc::L1Distance(m4.block(0, 0, 0, 0), m4.block(0, 0, 0, 0)), 0));
}

template<typename T>
void testPearsonCorrelationOCV() {
	auto m1 = cv::Mat(4, 4, cv::DataType<T>::type, cv::Scalar::all(0));
	auto m2 = cv::Mat(4, 4, cv::DataType<T>::type, cv::Scalar::all(1));

	expect(eq(dv::imgproc::pearsonCorrelation(m1, m2).has_value(), false));

	int8_t i = 0;
	for (uint8_t row = 0; row < m1.rows; row++) {
		for (uint8_t col = 0; col < m1.cols; col++) {
			m1.at<T>(row, col) = i++;
		}
	}

	m1.copyTo(m2);

	const auto corr = dv::imgproc::pearsonCorrelation(m1, m2);
	expect(eq(corr.has_value(), true));
	expect(eq(*corr, 1));

	int8_t j = m2.rows * m2.cols - 1;
	for (uint8_t row = 0; row < m2.rows; row++) {
		for (uint8_t col = 0; col < m2.cols; col++) {
			m2.at<T>(row, col) = j--;
		}
	}

	const auto corr2 = dv::imgproc::pearsonCorrelation(m1, m2);
	expect(eq(corr2.has_value(), true));
	expect(eq(*corr2, -1));

	const auto m3 = cv::Mat(0, 0, cv::DataType<T>::type);
	const auto m4 = cv::Mat(0, 0, cv::DataType<T>::type);

	const auto corr3 = dv::imgproc::pearsonCorrelation(m3, m4);
	expect(eq(corr3.has_value(), true));
	expect(eq(*corr3, 0));
}

template<typename T>
void testPearsonCorrelationEigen() {
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m1 = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(4, 4);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m2 = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(4, 4);

	expect(eq(dv::imgproc::pearsonCorrelation(m1, m2).has_value(), false));

	int8_t i = 0;
	for (uint8_t row = 0; row < m1.rows(); row++) {
		for (uint8_t col = 0; col < m1.cols(); col++) {
			m1(row, col) = i++;
		}
	}

	m2 = m1;

	const auto corr = dv::imgproc::pearsonCorrelation(m1.block(0, 0, 4, 4), m2.block(0, 0, 4, 4));
	expect(eq(corr.has_value(), true));
	expect(eq(*corr, 1));

	int8_t j = m2.rows() * m2.cols() - 1;
	for (uint8_t row = 0; row < m2.rows(); row++) {
		for (uint8_t col = 0; col < m2.cols(); col++) {
			m2(row, col) = j--;
		}
	}

	const auto corr2 = dv::imgproc::pearsonCorrelation(m1, m2);
	expect(eq(corr2.has_value(), true));
	expect(eq(*corr2, -1));

	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m3
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0);
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m4
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(0, 0);

	const auto corr3 = dv::imgproc::pearsonCorrelation(m3, m4);
	expect(eq(corr3.has_value(), true));
	expect(eq(*corr3, 0));
}

template<typename T>
void testCosineDistanceOCV() {
	auto m1 = cv::Mat(4, 4, cv::DataType<T>::type, cv::Scalar::all(0));
	auto m2 = cv::Mat(4, 4, cv::DataType<T>::type, cv::Scalar::all(1));

	expect(eq(dv::imgproc::cosineDistance(m1, m2).has_value(), false));

	int8_t i = 0;
	for (uint8_t row = 0; row < m1.rows; row++) {
		for (uint8_t col = 0; col < m1.cols; col++) {
			m1.at<T>(row, col) = i++;
		}
	}

	m1.copyTo(m2);

	const auto corr = dv::imgproc::cosineDistance(m1, m2);
	expect(eq(corr.has_value(), true));
	expect(eq(*corr, 1));

	if (std::is_signed_v<T>) {
		m1.convertTo(m2, cv::DataType<T>::type, -1.0, 0.0);

		const auto corr2 = dv::imgproc::cosineDistance(m1, m2);
		expect(eq(corr2.has_value(), true));
		expect(eq(*corr2, -1));
	}

	const auto m3 = cv::Mat(0, 0, cv::DataType<T>::type);
	const auto m4 = cv::Mat(0, 0, cv::DataType<T>::type);

	const auto corr3 = dv::imgproc::cosineDistance(m3, m4);
	expect(eq(corr3.has_value(), true));
	expect(eq(*corr3, 0));
}

template<typename T>
void testCosineDistanceEigen() {
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m1 = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(4, 4);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m2 = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(4, 4);

	expect(eq(dv::imgproc::cosineDistance(m1, m2).has_value(), false));

	int8_t i = 0;
	for (uint8_t row = 0; row < m1.rows(); row++) {
		for (uint8_t col = 0; col < m1.cols(); col++) {
			m1(row, col) = i++;
		}
	}

	m2 = m1;

	const auto corr = dv::imgproc::cosineDistance(m1, m2);
	expect(eq(corr.has_value(), true));
	expect(eq(*corr, 1));

	if (std::is_signed_v<T>) {
		m2 = -m1;

		const auto corr2 = dv::imgproc::cosineDistance(m1, m2);
		expect(eq(corr2.has_value(), true));
		expect(eq(*corr2, -1));
	}

	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m3
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0);
	const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> m4
		= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(0, 0);

	const auto corr3 = dv::imgproc::cosineDistance(m3, m4);
	expect(eq(corr3.has_value(), true));
	expect(eq(*corr3, 0));
}

int main() {
	"Eigen_L1Distance_uint8_t"_test = [] {
		testL1DistanceEigen<uint8_t>();
	};

	"Eigen_L1Distance_int8_t"_test = [] {
		testL1DistanceEigen<int8_t>();
	};

	"Eigen_L1Distance_uint16_t"_test = [] {
		testL1DistanceEigen<uint16_t>();
	};

	"Eigen_L1Distance_int16_t"_test = [] {
		testL1DistanceEigen<int16_t>();
	};

	"Eigen_L1Distance_uint32_t"_test = [] {
		testL1DistanceEigen<uint32_t>();
	};

	"Eigen_L1Distance_int32_t"_test = [] {
		testL1DistanceEigen<int32_t>();
	};

	"Eigen_L1Distance_uint64_t"_test = [] {
		testL1DistanceEigen<uint64_t>();
	};

	"Eigen_L1Distance_int64_t"_test = [] {
		testL1DistanceEigen<int64_t>();
	};

	"OpenCV_L1Distance_uint8_t"_test = [] {
		testL1DistanceOCV<uint8_t>();
	};

	"OpenCV_L1Distance_int8_t"_test = [] {
		testL1DistanceOCV<int8_t>();
	};

	"OpenCV_L1Distance_uint16_t"_test = [] {
		testL1DistanceOCV<uint16_t>();
	};

	"OpenCV_L1Distance_int16_t"_test = [] {
		testL1DistanceOCV<int16_t>();
	};

	"OpenCV_L1Distance_int32_t"_test = [] {
		testL1DistanceOCV<int32_t>();
	};

	"Eigen_pearsonCorrelation_uint8_t"_test = [] {
		testPearsonCorrelationEigen<uint8_t>();
	};

	"Eigen_pearsonCorrelation_int8_t"_test = [] {
		testPearsonCorrelationEigen<int8_t>();
	};

	"Eigen_pearsonCorrelation_uint16_t"_test = [] {
		testPearsonCorrelationEigen<uint16_t>();
	};

	"Eigen_pearsonCorrelation_int16_t"_test = [] {
		testPearsonCorrelationEigen<int16_t>();
	};

	"Eigen_pearsonCorrelation_uint32_t"_test = [] {
		testPearsonCorrelationEigen<uint32_t>();
	};

	"Eigen_pearsonCorrelation_int32_t"_test = [] {
		testPearsonCorrelationEigen<int32_t>();
	};

	"Eigen_pearsonCorrelation_uint64_t"_test = [] {
		testPearsonCorrelationEigen<uint64_t>();
	};

	"Eigen_pearsonCorrelation_int64_t"_test = [] {
		testPearsonCorrelationEigen<int64_t>();
	};

	"OpenCV_pearsonCorrelation_uint8_t"_test = [] {
		testPearsonCorrelationOCV<uint8_t>();
	};

	"OpenCV_pearsonCorrelation_int8_t"_test = [] {
		testPearsonCorrelationOCV<int8_t>();
	};

	"OpenCV_pearsonCorrelation_uint16_t"_test = [] {
		testPearsonCorrelationOCV<uint16_t>();
	};

	"OpenCV_pearsonCorrelation_int16_t"_test = [] {
		testPearsonCorrelationOCV<int16_t>();
	};

	"OpenCV_pearsonCorrelation_int32_t"_test = [] {
		testPearsonCorrelationOCV<int32_t>();
	};

	"Eigen_cosineDistance_uint8_t"_test = [] {
		testCosineDistanceEigen<uint8_t>();
	};

	"Eigen_cosineDistance_int8_t"_test = [] {
		testCosineDistanceEigen<int8_t>();
	};

	"Eigen_cosineDistance_uint16_t"_test = [] {
		testCosineDistanceEigen<uint16_t>();
	};

	"Eigen_cosineDistance_int16_t"_test = [] {
		testCosineDistanceEigen<int16_t>();
	};

	"Eigen_cosineDistance_uint32_t"_test = [] {
		testCosineDistanceEigen<uint32_t>();
	};

	"Eigen_cosineDistance_int32_t"_test = [] {
		testCosineDistanceEigen<int32_t>();
	};

	"Eigen_cosineDistance_uint64_t"_test = [] {
		testCosineDistanceEigen<uint64_t>();
	};

	"Eigen_cosineDistance_int64_t"_test = [] {
		testCosineDistanceEigen<int64_t>();
	};

	"OpenCV_cosineDistance_uint8_t"_test = [] {
		testCosineDistanceOCV<uint8_t>();
	};

	"OpenCV_cosineDistance_int8_t"_test = [] {
		testCosineDistanceOCV<int8_t>();
	};

	"OpenCV_cosineDistance_uint16_t"_test = [] {
		testCosineDistanceOCV<uint16_t>();
	};

	"OpenCV_cosineDistance_int16_t"_test = [] {
		testCosineDistanceOCV<int16_t>();
	};

	"OpenCV_cosineDistance_int32_t"_test = [] {
		testCosineDistanceOCV<int32_t>();
	};

	return EXIT_SUCCESS;
}

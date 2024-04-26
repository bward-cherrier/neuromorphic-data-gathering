#include "../../include/dv-processing/core/utils.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	"isWithinDimensions"_test = [] {
		const cv::Size resolution(100, 100);
		// Acceptable indices
		expect(dv::isWithinDimensions(cv::Point2i(0, 0), resolution));
		expect(dv::isWithinDimensions(cv::Point2i(99, 99), resolution));
		expect(dv::isWithinDimensions(cv::Point2f(50.f, 50.f), resolution));
		expect(dv::isWithinDimensions(cv::Point2f(0.f, 0.f), resolution));
		expect(dv::isWithinDimensions(cv::Point2f(99.f, 99.f), resolution));
		expect(dv::isWithinDimensions(cv::Point2d(50., 50.), resolution));

		// Out of range indices
		expect(!dv::isWithinDimensions(cv::Point2i(0, -1), resolution));
		expect(!dv::isWithinDimensions(cv::Point2i(100, 99), resolution));
		expect(!dv::isWithinDimensions(cv::Point2f(99.1f, 50.f), resolution));
		expect(!dv::isWithinDimensions(cv::Point2d(50., -0.1), resolution));
	};

	return EXIT_SUCCESS;
}

#include "../../include/dv-processing/core/event.hpp"
#include "../../include/dv-processing/core/frame.hpp"

#include "boost/ut.hpp"

#include <opencv2/imgproc.hpp>

cv::Rect cvBoundingRect(const cv::Size &resolution, const dv::EventStore &events) {
	dv::EdgeMapAccumulator accumulator(resolution, 1.f);
	accumulator.accept(events);
	return cv::boundingRect(accumulator.generateFrame().image);
}

int main() {
	using namespace boost::ut;

	"horizontal_line_bounds"_test = [] {
		dv::EventStore events;

		events.emplace_back(0, 5, 5, false);
		events.emplace_back(0, 6, 5, false);
		events.emplace_back(0, 7, 5, false);
		events.emplace_back(0, 8, 5, false);
		events.emplace_back(0, 9, 5, false);
		events.emplace_back(0, 10, 5, false);

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 5));
		expect(eq(rectangle.y, 5));
		expect(eq(rectangle.width, 6));
		expect(eq(rectangle.height, 1));
		expect(eq(rectangle.area(), 6));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};

	"vertical_line_bounds"_test = [&] {
		dv::EventStore events;

		events.emplace_back(0, 5, 5, false);
		events.emplace_back(0, 5, 6, false);
		events.emplace_back(0, 5, 7, false);
		events.emplace_back(0, 5, 8, false);

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 5));
		expect(eq(rectangle.y, 5));
		expect(eq(rectangle.width, 1));
		expect(eq(rectangle.height, 4));
		expect(eq(rectangle.area(), 4));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};

	"diagonal_line_bounds"_test = [&] {
		dv::EventStore events;

		events.emplace_back(0, 5, 5, false);
		events.emplace_back(0, 6, 6, false);
		events.emplace_back(0, 7, 7, false);
		events.emplace_back(0, 8, 8, false);

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 5));
		expect(eq(rectangle.y, 5));
		expect(eq(rectangle.width, 4));
		expect(eq(rectangle.height, 4));
		expect(eq(rectangle.area(), 16));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};

	"wide_corners_bounds"_test = [&] {
		dv::EventStore events;

		events.emplace_back(0, 0, 0, false);
		events.emplace_back(0, 99, 99, false);

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 0));
		expect(eq(rectangle.y, 0));
		expect(eq(rectangle.width, 100));
		expect(eq(rectangle.height, 100));
		expect(eq(rectangle.area(), 10000));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};

	"single_pixel_bounds"_test = [&] {
		dv::EventStore events;

		events.emplace_back(0, 5, 7, false);

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 5));
		expect(eq(rectangle.y, 7));
		expect(eq(rectangle.width, 1));
		expect(eq(rectangle.height, 1));
		expect(eq(rectangle.area(), 1));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};

	"empty_store_bounds"_test = [&] {
		dv::EventStore events;

		cv::Rect rectangle = dv::boundingRect(events);
		expect(eq(rectangle.x, 0));
		expect(eq(rectangle.y, 0));
		expect(eq(rectangle.width, 0));
		expect(eq(rectangle.height, 0));
		expect(eq(rectangle.area(), 0));

		// Check consistency with opencv
		cv::Rect cvRectangle = cvBoundingRect(cv::Size(100, 100), events);
		expect(eq(rectangle, cvRectangle));
	};
}

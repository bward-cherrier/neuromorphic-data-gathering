//
// Created by rokas on 11.08.21.
//

#include "../../include/dv-processing/data/timed_keypoint_base.hpp"
#include "../../include/dv-processing/data/utilities.hpp"

#include "boost/ut.hpp"

#include <opencv2/core.hpp>

int main() {
	using namespace boost::ut;

	dv::TimedKeyPoint keypoint;
	keypoint.pt        = dv::Point2f(1.f, 2.f);
	keypoint.size      = 1.5f;
	keypoint.angle     = 2.5f;
	keypoint.response  = 3.5f;
	keypoint.octave    = 0;
	keypoint.timestamp = 10000000;

	"keypoint_batch_conversion"_test = [&] {
		dv::cvector<dv::TimedKeyPoint> tKeyPoints(2, keypoint);
		auto points = dv::data::fromTimedKeyPoints(tKeyPoints);
		expect(eq(points.size(), tKeyPoints.size()));
		auto lastPoint = points.back();

		expect(eq(lastPoint.pt.x, keypoint.pt.x()));
		expect(eq(lastPoint.pt.y, keypoint.pt.y()));
		expect(eq(lastPoint.size, keypoint.size));
		expect(eq(lastPoint.angle, keypoint.angle));
		expect(eq(lastPoint.response, keypoint.response));
		expect(eq(lastPoint.octave, keypoint.octave));

		auto pointsBack = dv::data::fromCvKeypoints(points, keypoint.timestamp);
		expect(eq(pointsBack.back().pt.x(), keypoint.pt.x()));
		expect(eq(pointsBack.back().pt.y(), keypoint.pt.y()));
		expect(eq(pointsBack.back().size, keypoint.size));
		expect(eq(pointsBack.back().angle, keypoint.angle));
		expect(eq(pointsBack.back().response, keypoint.response));
		expect(eq(pointsBack.back().octave, keypoint.octave));

		auto floatPoints = dv::data::convertToCvPoints(tKeyPoints);
		expect(eq(floatPoints.back().x, keypoint.pt.x()));
		expect(eq(floatPoints.back().y, keypoint.pt.y()));
	};

	return EXIT_SUCCESS;
}

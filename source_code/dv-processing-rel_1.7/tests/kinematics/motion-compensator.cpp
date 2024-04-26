#include "../../include/dv-processing/core/event.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/kinematics/motion_compensator.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>
#include <fstream>
#include <numbers>

dv::Event rotateEventAroundZ(
	const cv::Point &referencePoint, float angle, const dv::Event &event, int64_t newTimestamp) {
	auto x_rot
		= ((event.x() - referencePoint.x) * std::cos(angle)) - ((event.y() - referencePoint.y) * std::sin(angle));
	auto y_rot
		= ((event.x() - referencePoint.x) * std::sin(angle)) + ((event.y() - referencePoint.y) * std::cos(angle));
	return dv::Event(newTimestamp, x_rot + referencePoint.x, y_rot + referencePoint.y, event.polarity());
}

int main(int ac, char **av) {
	using namespace boost::ut;
	using namespace dv::kinematics;

	bool showPreview = false;

	CLI::App app{"Motion compensator tests"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	"compensation"_test = [] {
		cv::Size resolution(100, 100);

		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);

		auto mc = MotionCompensator(camera, std::make_unique<dv::EdgeMapAccumulator>(resolution));

		dv::Frame frame = mc.generateFrame();
		// Black image
		expect(eq(frame.image.size(), resolution));
		expect(eq(cv::sum(frame.image)[0], 0.0));

		int64_t sampleTimestamp = 1000000LL;

		auto store = dv::data::generate::eventLine(
			sampleTimestamp, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1));

		mc.accept(store);
		frame = mc.generateFrame();

		MotionCompensator<>::Info info = mc.getInfo();

		// No data
		expect(eq(frame.image.size(), resolution));
		expect(eq(info.depthAvailable, false));
		expect(eq(info.generationTime, sampleTimestamp));
		expect(eq(info.imageCompensated, false));
		expect(eq(info.transformsAvailable, false));
		expect(eq(info.inputEventCount, store.size()));
		expect(eq(info.accumulatedEventCount, store.size()));
		expect(eq(info.depthTime, -1));

		mc.accept(dv::measurements::Depth(sampleTimestamp, 1.f));
		mc.accept(store);
		frame = mc.generateFrame();
		info  = mc.getInfo();

		expect(eq(frame.image.size(), resolution));
		expect(eq(info.depthAvailable, true));
		expect(eq(info.generationTime, sampleTimestamp));
		expect(eq(info.imageCompensated, false));
		expect(eq(info.transformsAvailable, false));
		expect(eq(info.inputEventCount, store.size()));
		expect(eq(info.accumulatedEventCount, store.size()));
		expect(eq(info.depthTime, sampleTimestamp));

		Eigen::Matrix<float, 4, 4> identity;
		identity.setIdentity();

		mc.accept(Transformationf(sampleTimestamp - 1000, identity));
		mc.accept(store);
		frame = mc.generateFrame();
		info  = mc.getInfo();
		// Not enough transforms, image should not be compensated
		expect(eq(frame.image.size(), resolution));
		expect(eq(info.depthAvailable, true));
		expect(eq(info.generationTime, sampleTimestamp));
		expect(eq(info.imageCompensated, false));
		expect(eq(info.transformsAvailable, false));
		expect(eq(info.inputEventCount, store.size()));
		expect(eq(info.depthTime, sampleTimestamp));

		// Assume no motion
		mc.accept(Transformationf(sampleTimestamp, identity));
		mc.accept(Transformationf(sampleTimestamp + 1000, identity));
		mc.accept(store);
		frame = mc.generateFrame();
		info  = mc.getInfo();

		expect(eq(frame.image.size(), resolution));
		expect(eq(info.depthAvailable, true));
		expect(eq(info.generationTime, sampleTimestamp));
		expect(eq(info.imageCompensated, true));
		expect(eq(info.transformsAvailable, true));
		expect(eq(info.inputEventCount, store.size()));
		expect(eq(info.depthTime, sampleTimestamp));
	};

	"rotation_compensation_frame_generation"_test = [showPreview] {
		if (showPreview) {
			cv::namedWindow("Preview", cv::WINDOW_NORMAL);
		}
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);

		dv::EventStore line
			= dv::data::generate::eventLine(10000LL, cv::Point(resolution.width * (1.f / 5.f), resolution.height / 2),
				cv::Point(resolution.width * (2.f / 5.f), resolution.height / 2));

		dv::EdgeMapAccumulator accumulator(resolution);

		float angle = std::numbers::pi_v<float> / 5.f;

		dv::kinematics::MotionCompensator<> mc(camera);
		dv::EventStore events;

		int numOfLines = 5;

		// We rotate a line along the Z axis every 100us
		dv::EventStore rotatedLine;
		for (int i = 0; i < numOfLines; i++) {
			rotatedLine        = dv::EventStore();
			float currentAngle = angle * static_cast<float>(i);
			int64_t timestamp  = line.getLowestTime() + (1000 * i);
			for (const auto &event : line) {
				rotatedLine.push_back(rotateEventAroundZ(camera->getCentralPoint(), currentAngle, event, timestamp));
			}
			events.add(rotatedLine);
		}

		accumulator.accept(events);
		dv::Frame accumulatedFrame = accumulator.generateFrame();

		if (showPreview) {
			cv::imshow("Preview", accumulatedFrame.image);
			cv::waitKey(0);
		}

		mc.accept(events);
		dv::Frame nonMotionCompensated = mc.generateFrame();

		expect(eq(cv::sum(accumulatedFrame.image - nonMotionCompensated.image)[0], 0.0));

		if (showPreview) {
			cv::imshow("Preview", nonMotionCompensated.image);
			cv::waitKey(0);
		}
		// Only add first and last rotation, the intermediate transforms must be interpolated internally.
		{
			// We add an according transform into a transformer
			Eigen::Matrix3f rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).toRotationMatrix();
			dv::kinematics::Transformationf start(events.getLowestTime(), Eigen::Matrix<float, 3, 1>::Zero(), rotation);
			mc.accept(start);
		}
		{
			Eigen::Matrix3f rotation
				= Eigen::AngleAxisf(static_cast<float>(numOfLines - 1) * -angle, Eigen::Vector3f::UnitZ())
					  .toRotationMatrix();
			dv::kinematics::Transformationf start(
				events.getHighestTime() + 1, Eigen::Matrix<float, 3, 1>::Zero(), rotation);
			mc.accept(start);
		}

		// Motion compensate into start position, lines should rotate back into initial position
		mc.accept(events);
		dv::Frame motionCompensatedToStart = mc.generateFrame(events.getLowestTime());
		expect(neq(cv::sum(motionCompensatedToStart.image)[0], 0.0));
		if (showPreview) {
			cv::imshow("Preview", motionCompensatedToStart.image);
			cv::waitKey(0);
		}
		auto roi = dv::boundingRect(line);
		// Buffer the ROI to account for transformation inaccuracies (it is expected due to approximate numbers)
		int bufferValue = 5;
		roi.x           -= bufferValue;
		roi.y           -= bufferValue;
		roi.width       += bufferValue * 2;
		roi.height      += bufferValue * 2;

		// Black out the compensated line
		motionCompensatedToStart.image(roi) = 0;

		expect(eq(cv::sum(motionCompensatedToStart.image)[0], 0.0));
		if (showPreview) {
			cv::imshow("Preview", motionCompensatedToStart.image);
			cv::waitKey(0);
		}
		mc.accept(events);
		dv::Frame motionCompensated = mc.generateFrame(events.getHighestTime());
		expect(neq(cv::sum(motionCompensated.image)[0], 0.0));
		if (showPreview) {
			cv::imshow("Preview", motionCompensated.image);
			cv::waitKey(0);
		}
		// rotatedLine contains the last generated line
		roi        = dv::boundingRect(rotatedLine);
		roi.x      -= bufferValue;
		roi.y      -= bufferValue;
		roi.width  += bufferValue * 2;
		roi.height += bufferValue * 2;

		// Black out the compensated line
		motionCompensated.image(roi) = 0;

		expect(eq(cv::sum(motionCompensated.image)[0], 0.0));
		if (showPreview) {
			cv::imshow("Preview", motionCompensated.image);
			cv::waitKey(0);
		}
	};

	"rotation_compensated_event_generation"_test = [showPreview] {
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);

		dv::EventStore line
			= dv::data::generate::eventLine(10000LL, cv::Point(resolution.width * (1.f / 5.f), resolution.height / 2),
				cv::Point(resolution.width * (2.f / 5.f), resolution.height / 2));

		float angle = std::numbers::pi_v<float> / 20.f;

		dv::kinematics::MotionCompensator<> mc(camera);
		dv::EventStore events;

		int numOfLines = 5;

		// We rotate a line along the Z axis every 100us
		dv::EventStore rotatedLine;
		for (int i = 0; i < numOfLines; i++) {
			rotatedLine        = dv::EventStore();
			float currentAngle = angle * static_cast<float>(i);
			int64_t timestamp  = line.getLowestTime() + (1000 * i);
			for (const auto &event : line) {
				rotatedLine.push_back(rotateEventAroundZ(camera->getCentralPoint(), currentAngle, event, timestamp));
			}
			events.add(rotatedLine);
		}

		// Only add first and last rotation, the intermediate transforms must be interpolated internally.
		{
			// We add an according transform into a transformer
			Eigen::Matrix3f rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).toRotationMatrix();
			dv::kinematics::Transformationf start(events.getLowestTime(), Eigen::Matrix<float, 3, 1>::Zero(), rotation);
			mc.accept(start);
		}
		{
			Eigen::Matrix3f rotation
				= Eigen::AngleAxisf(static_cast<float>(numOfLines - 1) * -angle, Eigen::Vector3f::UnitZ())
					  .toRotationMatrix();
			dv::kinematics::Transformationf start(
				events.getHighestTime() + 1, Eigen::Matrix<float, 3, 1>::Zero(), rotation);
			mc.accept(start);
		}

		// Motion compensate into start position, lines should rotate back into initial position
		mc.accept(events);
		dv::EventStore eventCompensatedStart = mc.generateEvents(events.getHighestTime());
		expect(gt(eventCompensatedStart.size(), 0));

		auto roiOuter = dv::boundingRect(rotatedLine);
		// Buffer the ROI to account for transformation inaccuracies (it is expected due to approximate numbers)
		int bufferValue = 5;
		roiOuter.x      -= bufferValue;
		roiOuter.y      -= bufferValue;
		roiOuter.width  += bufferValue * 2;
		roiOuter.height += bufferValue * 2;

		auto roiInner = dv::boundingRect(rotatedLine);
		// Buffer the ROI to account for transformation inaccuracies (it is expected due to approximate numbers)
		roiInner.x      += bufferValue;
		roiInner.y      += bufferValue;
		roiInner.width  -= bufferValue * 2;
		roiInner.height -= bufferValue * 2;

		auto roiCompensatedEvents = dv::boundingRect(eventCompensatedStart);
		// Test that top left corner is larger than outer and lower then inner
		expect(gt(roiCompensatedEvents.x, roiOuter.x));
		expect(gt(roiCompensatedEvents.y, roiOuter.y));
		expect(lt(roiCompensatedEvents.x, roiInner.x));
		expect(lt(roiCompensatedEvents.y, roiInner.y));

		// Test that bottom right corner is larger than inner and lower then outer
		expect(gt(roiCompensatedEvents.x + roiCompensatedEvents.width, roiInner.x + roiInner.width));
		expect(gt(roiCompensatedEvents.y + roiCompensatedEvents.height, roiInner.y + roiInner.height));
		expect(lt(roiCompensatedEvents.x + roiCompensatedEvents.width, roiOuter.x + roiOuter.width));
		expect(lt(roiCompensatedEvents.y + roiCompensatedEvents.height, roiOuter.y + roiOuter.height));
	};

	"constant_depth_setting"_test = [] {
		cv::Size resolution(100, 100);
		auto camera = std::make_shared<dv::camera::CameraGeometry>(100., 100., 50., 50., resolution);
		dv::kinematics::MotionCompensator<> mc(camera);

		// Default is 3.0 meters
		expect(eq(mc.getConstantDepth(), 3.0_f));

		mc.setConstantDepth(0.1f);
		expect(eq(mc.getConstantDepth(), 0.1_f));

		expect(throws([&mc] {
			mc.setConstantDepth(-1.f);
		}));
	};
}

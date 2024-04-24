#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/io/stereo_camera_recording.hpp"
#include "../../include/dv-processing/io/stereo_camera_writer.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

void writeSampleData(dv::io::MonoCameraWriter &writer, const cv::Size &resolution) {
	writer.writeFrame(dv::Frame(10'000'000, dv::data::generate::sampleImage(resolution)));
	writer.writeFrame(dv::Frame(15'000'000, dv::data::generate::sampleImage(resolution)));
	writer.writeFrame(dv::Frame(20'000'000, dv::data::generate::sampleImage(resolution)));

	writer.writeTrigger(dv::Trigger(10'000'000, dv::TriggerType::APS_FRAME_START));
	writer.writeTrigger(dv::Trigger(12'000'000, dv::TriggerType::APS_FRAME_START));
	writer.writeTrigger(dv::Trigger(14'000'000, dv::TriggerType::APS_FRAME_END));
	writer.writeTrigger(dv::Trigger(15'000'000, dv::TriggerType::APS_EXPOSURE_START));
	writer.writeTrigger(dv::Trigger(16'000'000, dv::TriggerType::APS_FRAME_START));
	writer.writeTrigger(dv::Trigger(17'000'000, dv::TriggerType::APS_EXPOSURE_END));
	writer.writeTrigger(dv::Trigger(18'000'000, dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE));
	writer.writeTrigger(dv::Trigger(20'000'000, dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE));

	writer.writeEvents(dv::data::generate::eventTestSet(10'000'000, resolution));
	writer.writeEvents(dv::data::generate::eventTestSet(15'000'000, resolution));
	writer.writeEvents(dv::data::generate::eventTestSet(20'000'000, resolution));

	writer.writeImu(dv::IMU(10'000'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
	writer.writeImu(dv::IMU(10'100'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
	writer.writeImu(dv::IMU(10'500'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));

	dv::TimedKeyPointPacket kpts;
	kpts.elements.emplace_back(dv::Point2f(1.f, 2.f), 3.f, 4.f, 5.f, 6, 7, 18'000'000);
	writer.writePacket(kpts, "keypoints");
}

int main() {
	namespace fs = std::filesystem;
	using namespace std::chrono_literals;

	"stereo_write_read"_test = [] {
		cv::Size resolution(100, 100);
		auto leftConfig  = dv::io::MonoCameraWriter::DAVISConfig("left", resolution);
		auto rightConfig = dv::io::MonoCameraWriter::DAVISConfig("right", resolution);
		leftConfig.addStream<dv::TimedKeyPointPacket>("keypoints");
		rightConfig.addStream<dv::TimedKeyPointPacket>("keypoints");
		fs::path filepath("./stereo_writer.aedat4");

		// Create and write in the stereo data
		{
			dv::io::StereoCameraWriter writer(filepath, leftConfig, rightConfig);

			writeSampleData(writer.left, resolution);
			writeSampleData(writer.right, resolution);
		}

		dv::io::StereoCameraRecording reader("./stereo_writer.aedat4", "left", "right");
		auto &left = reader.getLeftReader();
		expect(eq(left.getCameraName(), leftConfig.cameraName));
		expect(eq(left.getDuration().count(), 10'000'000));
		expect(eq(left.getEventResolution().value(), resolution));
		expect(eq(left.getFrameResolution().value(), resolution));
		expect(eq(left.isFrameStreamAvailable(), true));
		expect(eq(left.isEventStreamAvailable(), true));
		expect(eq(left.isImuStreamAvailable(), true));
		expect(eq(left.isTriggerStreamAvailable(), true));

		{
			auto frame = left.getNextFrame();
			expect(eq(frame.value().timestamp, 10'000'000));
			frame = left.getNextFrame();
			expect(eq(frame.value().timestamp, 15'000'000));
			frame = left.getNextFrame();
			expect(eq(frame.value().timestamp, 20'000'000));
			frame = left.getNextFrame();
			expect(!frame.has_value());

			auto events = left.getNextEventBatch();
			expect(eq(events.value().getLowestTime(), 10'000'000));
			expect(eq(events.value().getHighestTime(), 10'000'000));
			events = left.getNextEventBatch();
			expect(eq(events.value().getLowestTime(), 15'000'000));
			expect(eq(events.value().getHighestTime(), 15'000'000));
			events = left.getNextEventBatch();
			expect(eq(events.value().getLowestTime(), 20'000'000));
			expect(eq(events.value().getHighestTime(), 20'000'000));
			events = left.getNextEventBatch();
			expect(!events.has_value());

			auto triggers = left.getNextTriggerBatch();
			expect(triggers.has_value());
			expect(eq(triggers.value().size(), 8));

			auto imus = left.getNextImuBatch();
			expect(imus.has_value());
			expect(eq(imus.value().size(), 3));
		}

		auto &right = reader.getRightReader();
		expect(eq(right.getCameraName(), rightConfig.cameraName));
		expect(eq(right.getDuration().count(), 10'000'000));
		expect(eq(right.getEventResolution().value(), resolution));
		expect(eq(right.getFrameResolution().value(), resolution));
		expect(eq(right.isFrameStreamAvailable(), true));
		expect(eq(right.isEventStreamAvailable(), true));
		expect(eq(right.isImuStreamAvailable(), true));
		expect(eq(right.isTriggerStreamAvailable(), true));

		auto frame = right.getNextFrame();
		expect(eq(frame.value().timestamp, 10'000'000));
		frame = right.getNextFrame();
		expect(eq(frame.value().timestamp, 15'000'000));
		frame = right.getNextFrame();
		expect(eq(frame.value().timestamp, 20'000'000));
		frame = right.getNextFrame();
		expect(!frame.has_value());

		auto events = right.getNextEventBatch();
		expect(eq(events.value().getLowestTime(), 10'000'000));
		expect(eq(events.value().getHighestTime(), 10'000'000));
		events = right.getNextEventBatch();
		expect(eq(events.value().getLowestTime(), 15'000'000));
		expect(eq(events.value().getHighestTime(), 15'000'000));
		events = right.getNextEventBatch();
		expect(eq(events.value().getLowestTime(), 20'000'000));
		expect(eq(events.value().getHighestTime(), 20'000'000));
		events = right.getNextEventBatch();
		expect(!events.has_value());

		auto triggers = right.getNextTriggerBatch();
		expect(triggers.has_value());
		expect(eq(triggers.value().size(), 8));

		auto imus = right.getNextImuBatch();
		expect(imus.has_value());
		expect(eq(imus.value().size(), 3));

		expect(left.isStreamAvailable("keypoints"));
		auto kpts = left.getNextStreamPacket<dv::TimedKeyPointPacket>("keypoints");
		expect(kpts.has_value());
		expect(eq(kpts.value().elements.size(), 1));
		const auto &kpt = kpts.value().elements.front();
		expect(eq(kpt.pt.x(), 1.0_f));
		expect(eq(kpt.pt.y(), 2.0_f));
		expect(eq(kpt.size, 3.0_f));
		expect(eq(kpt.angle, 4.0_f));
		expect(eq(kpt.response, 5.0_f));
		expect(eq(kpt.octave, 6));
		expect(eq(kpt.class_id, 7));
		expect(eq(kpt.timestamp, 18'000'000));
	};
}

#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/io/mono_camera_recording.hpp"
#include "../../include/dv-processing/io/mono_camera_writer.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	"improper_config"_test = [] {
		expect(nothrow([]() {
			dv::io::MonoCameraWriter writer(
				"./mono_writer.aedat4", dv::io::MonoCameraWriter::EventOnlyConfig("test", cv::Size(100, 100)));

			expect(writer.isEventStreamConfigured());
			expect(!writer.isFrameStreamConfigured());
			expect(!writer.isTriggerStreamConfigured());
			expect(!writer.isImuStreamConfigured());
		}));

		expect(throws([] {
			// Missing camera name
			dv::io::MonoCameraWriter::Config config("");
		}));

		expect(throws([] {
			// No streams enabled
			dv::io::MonoCameraWriter::Config config("test");
			dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		}));

		expect(throws([] {
			// Invalid resolution info
			dv::io::MonoCameraWriter::Config config("test");
			config.addEventStream(cv::Size(0, 0));
			dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		}));

		expect(throws([] {
			// Enable events with custom data type without providing resolution
			dv::io::MonoCameraWriter::Config config("test");
			config.addStream<dv::EventPacket>("events");
			dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		}));

		expect(throws([] {
			// Enable frame but only set event resolution
			dv::io::MonoCameraWriter::Config config("test");
			config.addStream<dv::Frame>("frames");
			dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		}));
	};

	"unconfigured_inputs_event"_test = [] {
		dv::io::MonoCameraWriter writer(
			"./mono_writer.aedat4", dv::io::MonoCameraWriter::EventOnlyConfig("test", cv::Size(100, 100)));
		expect(nothrow([&writer]() {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e);
			writer.writeEventPacket(dv::EventPacket());
			writer.writeEvents(dv::EventStore());

			expect(writer.isEventStreamConfigured());
			expect(!writer.isFrameStreamConfigured());
			expect(!writer.isTriggerStreamConfigured());
			expect(!writer.isImuStreamConfigured());
		}));

		// Not ok to write events with non-configured stream name
		expect(throws([&writer] {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e, "This name is definitely wrong");
		}));

		// Not ok to write events with non-configured stream name
		expect(throws([&writer] {
			dv::EventPacket e;
			e.elements.emplace_back(1, 1, 1, true);
			writer.writeEventPacket(e, "This name is definitely wrong");
		}));
		expect(throws([&writer]() {
			writer.writeImu(dv::IMU());
		}));
		expect(throws([&writer]() {
			writer.writeImuPacket(dv::IMUPacket());
		}));
		expect(throws([&writer]() {
			writer.writeTrigger(dv::Trigger());
		}));
		expect(throws([&writer]() {
			writer.writeTriggerPacket(dv::TriggerPacket());
		}));
	};

	"unconfigured_inputs_frame"_test = [] {
		const cv::Size resolution(100, 100);
		dv::io::MonoCameraWriter writer(
			"./mono_writer.aedat4", dv::io::MonoCameraWriter::FrameOnlyConfig("test", resolution));
		expect(nothrow([&writer, &resolution]() {
			writer.writeFrame(dv::Frame(0, cv::Mat(resolution, CV_8UC1, cv::Scalar(0))));
			// It is ok to pass empty frame, it will be ignored
			writer.writeFrame(dv::Frame(0, cv::Mat()));
			// It is ok write images with lower resolution than configured
			writer.writeFrame(dv::Frame(0, cv::Mat(cv::Size(50, 50), CV_8UC1, cv::Scalar(0))));

			expect(!writer.isEventStreamConfigured());
			expect(writer.isFrameStreamConfigured());
			expect(!writer.isTriggerStreamConfigured());
			expect(!writer.isImuStreamConfigured());
		}));

		// Not ok to write frame with higher dimensions than configured
		expect(throws([&writer] {
			writer.writeFrame(dv::Frame(0, cv::Mat(cv::Size(150, 50), CV_8UC1, cv::Scalar(0))));
		}));
		expect(throws([&writer] {
			writer.writeFrame(dv::Frame(0, cv::Mat(cv::Size(150, 150), CV_8UC1, cv::Scalar(0))));
		}));
		expect(throws([&writer] {
			writer.writeFrame(dv::Frame(0, cv::Mat(cv::Size(150, 150), CV_8UC1, cv::Scalar(0))));
		}));

		// Not ok to write frame with non-configured stream name
		expect(throws([&writer] {
			writer.writeFrame(
				dv::Frame(0, cv::Mat(cv::Size(150, 150), CV_8UC1, cv::Scalar(0))), "This name is definitely wrong");
		}));

		expect(nothrow([&writer]() {
			// Empty event store can pass
			writer.writeEvents(dv::EventStore());
		}));
		expect(throws([&writer]() {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e);
		}));
		expect(nothrow([&writer]() {
			writer.writeEventPacket(dv::EventPacket());
		}));
		expect(throws([&writer]() {
			writer.writeImu(dv::IMU());
		}));
		expect(throws([&writer]() {
			writer.writeImuPacket(dv::IMUPacket());
		}));
		expect(throws([&writer]() {
			writer.writeTrigger(dv::Trigger());
		}));
		expect(throws([&writer]() {
			writer.writeTriggerPacket(dv::TriggerPacket());
		}));
	};

	"unconfigured_inputs_imu"_test = [] {
		dv::io::MonoCameraWriter::Config config("test");
		config.addImuStream();

		dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		expect(nothrow([&writer]() {
			writer.writeImu(dv::IMU());
			writer.writeImuPacket(dv::IMUPacket());
		}));

		expect(!writer.isEventStreamConfigured());
		expect(!writer.isFrameStreamConfigured());
		expect(!writer.isTriggerStreamConfigured());
		expect(writer.isImuStreamConfigured());

		expect(throws([&writer]() {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e);
		}));
		expect(nothrow([&writer]() {
			writer.writeEventPacket(dv::EventPacket());
		}));
		expect(nothrow([&writer]() {
			writer.writeFrame(dv::Frame(0, cv::Mat()));
		}));
		expect(throws([&writer]() {
			writer.writeTrigger(dv::Trigger());
		}));
		expect(throws([&writer]() {
			writer.writeTriggerPacket(dv::TriggerPacket());
		}));
	};

	"unconfigured_inputs_trigger"_test = [] {
		dv::io::MonoCameraWriter::Config config("test");
		config.addTriggerStream();

		dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
		expect(nothrow([&writer]() {
			writer.writeTrigger(dv::Trigger());
			writer.writeTriggerPacket(dv::TriggerPacket());
		}));
		expect(!writer.isEventStreamConfigured());
		expect(!writer.isFrameStreamConfigured());
		expect(writer.isTriggerStreamConfigured());
		expect(!writer.isImuStreamConfigured());

		expect(throws([&writer]() {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e);
		}));
		expect(throws([&writer]() {
			dv::EventStore e;
			e.emplace_back(1, 1, 1, true);
			writer.writeEvents(e);
		}));
		expect(nothrow([&writer]() {
			writer.writeFrame(dv::Frame(0, cv::Mat()));
		}));
		expect(throws([&writer]() {
			writer.writeImu(dv::IMU());
		}));
		expect(throws([&writer]() {
			writer.writeImuPacket(dv::IMUPacket());
		}));
	};

	"out_of_order_timestamps"_test = [] {
		const cv::Size resolution(100, 100);
		dv::io::MonoCameraWriter::Config config = dv::io::MonoCameraWriter::DAVISConfig("test", resolution);
		dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);

		expect(eq(config.findStreamResolution("events").value(), resolution));
		expect(eq(config.findStreamResolution("frames").value(), resolution));

		dv::EventStore events = dv::data::generate::eventTestSet(10'000'000, resolution);
		writer.writeEvents(events);

		expect(throws([&writer, &config, &resolution] {
			writer.writeEvents(dv::data::generate::eventTestSet(1'000'000, resolution));
		}));

		expect(nothrow([&writer, &config, &resolution] {
			writer.writeEvents(dv::data::generate::eventTestSet(11'000'000, resolution));
		}));

		expect(throws([&writer] {
			// Bad timestamping within packet
			dv::EventStore packet;
			packet.emplace_back(20'000'000, 0, 0, false);
			packet.emplace_back(20'100'000, 0, 0, false);
			packet.emplace_back(500'000, 0, 0, true);

			writer.writeEvents(packet);
		}));

		expect(nothrow([&writer] {
			// Correct timestamping within packet
			dv::EventStore packet;
			packet.emplace_back(20'000'000, 0, 0, false);
			packet.emplace_back(20'100'000, 0, 0, false);
			packet.emplace_back(20'100'000, 0, 0, false);

			writer.writeEvents(packet);
		}));

		expect(throws([&writer] {
			// Bad timestamping within packet
			dv::IMUPacket packet;
			packet.elements.emplace_back(1'000'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			packet.elements.emplace_back(1'100'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			packet.elements.emplace_back(500'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

			writer.writeImuPacket(packet);
		}));

		expect(nothrow([&writer] {
			// Correct timestamping within packet
			dv::IMUPacket packet;
			packet.elements.emplace_back(1'000'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			packet.elements.emplace_back(1'100'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			packet.elements.emplace_back(1'200'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

			writer.writeImuPacket(packet);
		}));

		expect(throws([&writer] {
			writer.writeImu(dv::IMU(500'000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
		}));

		writer.writeFrame(dv::Frame(1'000'000, cv::Mat(resolution, CV_8UC1)));
		// The same timestamp should work just fine
		expect(nothrow([&writer, &config, &resolution] {
			writer.writeFrame(dv::Frame(1'000'000, cv::Mat(resolution, CV_8UC1)));
		}));
		// Past timestamp throws
		expect(throws([&writer, &config, &resolution] {
			writer.writeFrame(dv::Frame(500'000, cv::Mat(resolution, CV_8UC1)));
		}));

		expect(throws([&writer] {
			// Bad timestamping within packet
			dv::TriggerPacket packet;
			packet.elements.emplace_back(1'000'000, dv::TriggerType::APS_EXPOSURE_END);
			packet.elements.emplace_back(1'100'000, dv::TriggerType::APS_EXPOSURE_END);
			packet.elements.emplace_back(500'000, dv::TriggerType::APS_EXPOSURE_END);

			writer.writeTriggerPacket(packet);
		}));

		expect(nothrow([&writer] {
			// Correct timestamping within packet
			dv::TriggerPacket packet;
			packet.elements.emplace_back(1'000'000, dv::TriggerType::APS_EXPOSURE_END);
			packet.elements.emplace_back(1'100'000, dv::TriggerType::APS_EXPOSURE_END);
			packet.elements.emplace_back(1'200'000, dv::TriggerType::APS_EXPOSURE_END);

			writer.writeTriggerPacket(packet);
		}));

		expect(nothrow([&writer] {
			writer.writeTrigger(dv::Trigger(1'300'000, dv::TriggerType::APS_EXPOSURE_START));
		}));
		expect(throws([&writer] {
			writer.writeTrigger(dv::Trigger(500'000, dv::TriggerType::APS_EXPOSURE_START));
		}));
	};

	"full_write_read"_test = [] {
		const cv::Size resolution(100, 100);
		dv::io::MonoCameraWriter::Config config = dv::io::MonoCameraWriter::DAVISConfig("test", resolution);
		config.addStream<dv::Pose>("pose");

		size_t totalEventCount = 0;
		{
			// Scope this to trigger desctructor and finalize the recording
			dv::io::MonoCameraWriter writer("./mono_writer.aedat4", config);
			// Set packaging count to 4, so that IMU and triggers would be batched into 4
			writer.setPackagingCount(4);
			expect(writer.isEventStreamConfigured());
			expect(writer.isFrameStreamConfigured());
			expect(writer.isTriggerStreamConfigured());
			expect(writer.isImuStreamConfigured());

			dv::EventStore events = dv::data::generate::eventTestSet(10'000'000, resolution);
			totalEventCount       += events.size();
			writer.writeEvents(events);
			events          = dv::data::generate::eventTestSet(20'000'000, resolution);
			totalEventCount += events.size();
			writer.writeEvents(events);

			writer.writeFrame(dv::Frame(10'000'000, dv::data::generate::sampleImage(resolution)));
			writer.writeFrame(dv::Frame(20'000'000, dv::data::generate::sampleImage(resolution)));

			// 6 measurements - 4 must be batched and 2 must be dumped with destructor
			writer.writeImu(dv::IMU(10'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));
			writer.writeImu(dv::IMU(12'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));
			writer.writeImu(dv::IMU(14'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));
			writer.writeImu(dv::IMU(16'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));
			writer.writeImu(dv::IMU(18'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));
			writer.writeImu(dv::IMU(20'000'000, 20.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f));

			writer.writeTrigger(dv::Trigger(10'000'000, dv::TriggerType::APS_EXPOSURE_END));
			writer.writeTrigger(dv::Trigger(12'000'000, dv::TriggerType::APS_EXPOSURE_START));
			writer.writeTrigger(dv::Trigger(14'000'000, dv::TriggerType::EXTERNAL_SIGNAL_PULSE));
			writer.writeTrigger(dv::Trigger(16'000'000, dv::TriggerType::EXTERNAL_SIGNAL_RISING_EDGE));
			writer.writeTrigger(dv::Trigger(18'000'000, dv::TriggerType::EXTERNAL_GENERATOR_FALLING_EDGE));
			writer.writeTrigger(dv::Trigger(20'000'000, dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE));

			writer.writePacket(dv::Pose(10'000'000, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f),
								   "source", "target"),
				"pose");
			writer.writePacket(dv::Pose(12'000'000, dv::Vec3f(0.1f, 0.2f, 0.3f),
								   dv::Quaternion(0.525322f, 0.f, 0.8509035f, 0.f), "other", "another"),
				"pose");
			writer.writePacket(dv::Pose(14'000'000, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f),
								   "source", "target"),
				"pose");
		}

		dv::io::MonoCameraRecording reader("./mono_writer.aedat4");
		expect(eq(reader.getCameraName(), config.cameraName));
		auto timerange = reader.getTimeRange();
		expect(eq(timerange.first, 10'000'000));
		expect(eq(timerange.second, 20'000'000));

		expect(reader.isEventStreamAvailable());
		expect(eq(reader.getEventResolution().value(), resolution));

		dv::EventStore allEvents;
		while (const auto events = reader.getNextEventBatch()) {
			allEvents.add(events.value());
		}
		expect(eq(allEvents.size(), totalEventCount));
		expect(eq(allEvents.getLowestTime(), 10'000'000));
		expect(eq(allEvents.getHighestTime(), 20'000'000));

		expect(reader.isFrameStreamAvailable());
		expect(eq(reader.getFrameResolution().value(), resolution));

		dv::Frame frame1 = reader.getNextFrame().value();
		expect(eq(frame1.timestamp, 10'000'000));
		dv::Frame frame2 = reader.getNextFrame().value();
		expect(eq(frame2.timestamp, 20'000'000));
		// End of stream expected
		expect(!reader.getNextFrame().has_value());

		auto triggers1 = reader.getNextTriggerBatch().value();
		expect(eq(triggers1[0].timestamp, 10'000'000));
		expect(triggers1[0].type == dv::TriggerType::APS_EXPOSURE_END);
		expect(eq(triggers1[1].timestamp, 12'000'000));
		expect(triggers1[1].type == dv::TriggerType::APS_EXPOSURE_START);
		expect(eq(triggers1[2].timestamp, 14'000'000));
		expect(triggers1[2].type == dv::TriggerType::EXTERNAL_SIGNAL_PULSE);
		expect(eq(triggers1[3].timestamp, 16'000'000));
		expect(triggers1[3].type == dv::TriggerType::EXTERNAL_SIGNAL_RISING_EDGE);
		expect(eq(triggers1.size(), 4));
		auto triggers2 = reader.getNextTriggerBatch().value();
		expect(eq(triggers2.size(), 2));
		expect(eq(triggers2[0].timestamp, 18'000'000));
		expect(triggers2[0].type == dv::TriggerType::EXTERNAL_GENERATOR_FALLING_EDGE);
		expect(eq(triggers2[1].timestamp, 20'000'000));
		expect(triggers2[1].type == dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE);
		// End of stream expected
		expect(!reader.getNextTriggerBatch().has_value());

		auto imuBatch1 = reader.getNextImuBatch().value();
		expect(eq(imuBatch1.size(), 4));
		auto imuBatch2 = reader.getNextImuBatch().value();
		expect(eq(imuBatch2.size(), 2));
		// End of stream expected
		expect(!reader.getNextImuBatch().has_value());

		expect(reader.isStreamAvailable("pose"));
		auto pose1 = reader.getNextStreamPacket<dv::Pose>("pose").value();
		expect(eq(pose1.timestamp, 10'000'000));
		expect(eq(pose1.referenceFrame.c_str(), std::string("source")));
		expect(eq(pose1.targetFrame.c_str(), std::string("target")));
		expect(eq(pose1.translation.x(), .1f));
		expect(eq(pose1.translation.y(), .2f));
		expect(eq(pose1.translation.z(), .3f));
		expect(eq(pose1.rotation.w(), 1.f));
		expect(eq(pose1.rotation.x(), .0f));
		expect(eq(pose1.rotation.y(), .0f));
		expect(eq(pose1.rotation.z(), .0f));

		auto pose2 = reader.getNextStreamPacket<dv::Pose>("pose").value();
		expect(eq(pose2.timestamp, 12'000'000));
		expect(eq(pose2.referenceFrame.c_str(), std::string("other")));
		expect(eq(pose2.targetFrame.c_str(), std::string("another")));
		expect(eq(pose2.translation.x(), .1f));
		expect(eq(pose2.translation.y(), .2f));
		expect(eq(pose2.translation.z(), .3f));
		expect(eq(pose2.rotation.w(), .525322f));
		expect(eq(pose2.rotation.x(), .0f));
		expect(eq(pose2.rotation.y(), .8509035f));
		expect(eq(pose2.rotation.z(), .0f));

		auto pose3 = reader.getNextStreamPacket<dv::Pose>("pose").value();
		expect(eq(pose3.timestamp, 14'000'000));
		expect(eq(pose3.referenceFrame.c_str(), std::string("source")));
		expect(eq(pose3.targetFrame.c_str(), std::string("target")));
		expect(eq(pose3.translation.x(), .1f));
		expect(eq(pose3.translation.y(), .2f));
		expect(eq(pose3.translation.z(), .3f));
		expect(eq(pose3.rotation.w(), 1.f));
		expect(eq(pose3.rotation.x(), .0f));
		expect(eq(pose3.rotation.y(), .0f));
		expect(eq(pose3.rotation.z(), .0f));
		expect(!reader.getNextStreamPacket<dv::Pose>("pose").has_value());

		auto poses = reader.getStreamTimeRange<dv::Pose>(12'000'000, 14'000'000, "pose");
		expect(eq(poses->size(), 1));
		expect(eq(poses->at(0).rotation.w(), .525322f));
		expect(eq(poses->at(0).rotation.y(), .8509035f));
	};

	"custom_datatypes"_test = [] {
		dv::io::MonoCameraWriter::Config config("test");
		config.addStream<dv::TimedKeyPointPacket>("points");

		{
			dv::io::MonoCameraWriter writer("./custom_data.aedat4", config);

			dv::TimedKeyPointPacket datapacket;
			datapacket.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 100'000));
			datapacket.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 110'000));
			datapacket.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 120'000));
			datapacket.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 130'000));
			dv::TimedKeyPointPacket datapacket2;
			datapacket2.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 140'000));
			datapacket2.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 150'000));
			datapacket2.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 160'000));
			datapacket2.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 170'000));
			dv::TimedKeyPointPacket datapacket3;
			datapacket3.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 180'000));
			datapacket3.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 190'000));
			datapacket3.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 200'000));
			datapacket3.elements.emplace_back(dv::TimedKeyPoint(dv::Point2f(0.f, 0.f), 1.f, 0.f, 1.f, 0, 1, 210'000));
			expect(nothrow([&writer, &datapacket, &datapacket2, &datapacket3] {
				writer.writePacket(datapacket, "points");
				writer.writePacket(datapacket2, "points");
				writer.writePacket(datapacket3, "points");
			}));

			expect(throws([&writer, &datapacket] {
				writer.writePacket(datapacket, "poin___");
			}));
		}

		dv::io::MonoCameraRecording reader("./custom_data.aedat4");
		expect(eq(reader.getCameraName(), config.cameraName));
		expect(reader.isStreamAvailable("points"));
		expect(!reader.isStreamAvailable("points2"));
		expect(!reader.isStreamAvailable(""));

		const auto names = reader.getStreamNames();
		expect(eq(names.size(), 1));
		expect(eq(names.at(0), std::string("points")));

		const auto points = reader.getNextStreamPacket<dv::TimedKeyPointPacket>("points");
		expect(points.has_value());
		expect(eq(points.value().elements.size(), 4));
		expect(eq(points.value().elements.front().timestamp, 100'000));
		expect(eq(points.value().elements.front().size, 1.0_f));
		expect(eq(points.value().elements.front().angle, 0.0_f));
		expect(eq(points.value().elements.front().response, 1.0_f));
		expect(eq(points.value().elements.front().pt.x(), 0.0_f));
		expect(eq(points.value().elements.front().pt.y(), 0.0_f));
		expect(eq(points.value().elements.front().octave, 0));
		expect(eq(points.value().elements.front().class_id, 1));

		expect(reader.getNextStreamPacket<dv::TimedKeyPointPacket>("points").has_value());
		expect(reader.getNextStreamPacket<dv::TimedKeyPointPacket>("points").has_value());
		expect(!reader.getNextStreamPacket<dv::TimedKeyPointPacket>("points").has_value());

		const auto packets = reader.getStreamTimeRange<dv::TimedKeyPointPacket>(120'000, 200'000, "points");
		expect(packets.has_value());
		expect(!packets.value().empty());
		expect(eq(packets.value().size(), 3));
		expect(eq(packets.value().at(0).elements.size(), 2));
		expect(eq(packets.value().at(0).elements.front().timestamp, 120'000));
		expect(eq(packets.value().at(1).elements.size(), 4));
		expect(eq(packets.value().at(2).elements.size(), 2));
		expect(eq(packets.value().at(2).elements.back().timestamp, 190'000));
	};

	"frame_as_custom_datatype"_test = [] {
		cv::Size resolution(100, 100);

		dv::io::MonoCameraWriter::Config config("test");
		config.addFrameStream(resolution, "frames");

		expect(throws([&config, &resolution] {
			// Stream name cannot be empty
			config.addFrameStream(resolution, "");
		}));

		{
			dv::io::MonoCameraWriter writer("./custom_data_frame.aedat4", config);
			writer.writePacket(dv::Frame(1'000'000, dv::data::generate::sampleImage(resolution)), "frames");
			writer.writePacket(dv::Frame(2'000'000, dv::data::generate::sampleImage(resolution)), "frames");
			writer.writePacket(dv::Frame(3'000'000, dv::data::generate::sampleImage(resolution)), "frames");
		}

		dv::io::MonoCameraRecording reader("./custom_data_frame.aedat4");
		expect(eq(reader.getCameraName(), config.cameraName));
		expect(reader.isFrameStreamAvailable());
		expect(reader.isStreamAvailable("frames"));
		expect(!reader.isStreamAvailable("points2"));
		expect(!reader.isStreamAvailable(""));
		expect(nothrow([&reader, &resolution] {
			auto frameResolution = reader.getFrameResolution();
			expect(eq(frameResolution.value().width, resolution.width));
			expect(eq(frameResolution.value().height, resolution.height));
		}));

		// Now reading must succeed even though we write using custom data type
		auto frame1 = reader.getNextFrame();
		expect(frame1.has_value());
		expect(eq(frame1.value().image.size(), resolution));
		expect(eq(frame1.value().timestamp, 1'000'000));
		auto frame2 = reader.getNextFrame();
		expect(frame2.has_value());
		expect(eq(frame2.value().image.size(), resolution));
		expect(eq(frame2.value().timestamp, 2'000'000));
		auto frame3 = reader.getNextFrame();
		expect(frame3.has_value());
		expect(eq(frame3.value().image.size(), resolution));
		expect(eq(frame3.value().timestamp, 3'000'000));
		auto frame4 = reader.getNextFrame();
		expect(!frame4.has_value());
	};

	expect(nothrow([]() {
		dv::io::MonoCameraWriter writer(
			"mono_writer_test.aedat4", dv::io::MonoCameraWriter::EventOnlyConfig("test", cv::Size(100, 100)));
		expect(std::filesystem::exists("mono_writer_test.aedat4"));
	}));

	return EXIT_SUCCESS;
}

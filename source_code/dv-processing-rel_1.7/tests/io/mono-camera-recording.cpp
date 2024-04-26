#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/mono_camera_recording.hpp"
#include "../../include/dv-processing/io/mono_camera_writer.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	std::string file = "./test_files/test-minimal.aedat4";

	"stream_existence"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isEventStreamAvailable());
		expect(reader.isImuStreamAvailable());
		expect(!reader.isFrameStreamAvailable());
		expect(reader.isTriggerStreamAvailable());
	};

	"read_sequential"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isEventStreamAvailable());

		int64_t startTimestamp    = -1;
		int64_t lastReadTimestamp = -1;
		// Read until file end
		while (auto events = reader.getNextEventBatch()) {
			expect(ge(events->getLowestTime(), lastReadTimestamp));

			// Extract start time stamp for future reset test
			if (startTimestamp < 0) {
				startTimestamp = events->getLowestTime();
			}

			lastReadTimestamp = events->getHighestTime();
		}

		expect(reader.isTriggerStreamAvailable());
		while (auto triggers = reader.getNextTriggerBatch()) {
			expect(!triggers->empty());
		}

		lastReadTimestamp    = -1;
		int64_t imuStartTime = -1;
		expect(reader.isImuStreamAvailable());
		while (auto imu = reader.getNextImuBatch()) {
			expect(!imu->empty());
			expect(ge(imu->front().timestamp, lastReadTimestamp));

			if (imuStartTime < 0) {
				imuStartTime = imu->front().timestamp;
			}

			lastReadTimestamp = imu->back().timestamp;
		}

		// We should have exhausted the input data
		expect(!reader.getNextEventBatch().has_value());

		// Resetting the reader should allow us to read from start again
		reader.resetSequentialRead();
		auto events = reader.getNextEventBatch();
		expect(events.has_value());
		expect(eq(events->getLowestTime(), startTimestamp));

		// No frame stream is expected, std::nullopt should be returned
		expect(!reader.isFrameStreamAvailable());
		expect(throws([&reader] {
			auto frame = reader.getNextFrame();
		}));
	};

	"time_range_reading"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isEventStreamAvailable());
		expect(reader.isImuStreamAvailable());
		expect(reader.isTriggerStreamAvailable());
		expect(!reader.isFrameStreamAvailable());

		const auto range = reader.getTimeRange();
		// First is start, second is end, so it should have a lower numeric value
		expect(lt(range.first, range.second));

		// Reading the types
		const auto events = reader.getEventsTimeRange(range.first, range.second + 1);
		expect(events.has_value());
		expect(gt(events->size(), 0ULL));
		expect(ge(events->getLowestTime(), range.first));
		expect(le(events->getHighestTime(), range.second));

		const auto imu = reader.getImuTimeRange(range.first, range.second);
		expect(imu.has_value());
		expect(gt(imu->size(), 0ULL));
		expect(ge(imu->front().timestamp, range.first));
		expect(le(imu->back().timestamp, range.second));

		const auto triggers = reader.getTriggersTimeRange(range.first, range.second);
		expect(triggers.has_value());
		expect(triggers->empty());

		expect(throws([&reader, &range] {
			const auto frames = reader.getFramesTimeRange(range.first, range.second);
		}));

		// Invalid range reading
		auto noevents = reader.getEventsTimeRange(-10000, -1000);
		expect(noevents.has_value());
		expect(eq(noevents->size(), 0));

		noevents = reader.getEventsTimeRange(range.first - 10000, range.first);
		expect(noevents.has_value());
		expect(eq(noevents->size(), 0));

		noevents = reader.getEventsTimeRange(range.second + 1, range.second + 10000);
		expect(noevents.has_value());
		expect(eq(noevents->size(), 0));

		auto noimu = reader.getImuTimeRange(-10000, -1000);
		expect(noimu.has_value());
		expect(eq(noimu->size(), 0));

		noimu = reader.getImuTimeRange(range.first - 10000, range.first);
		expect(noimu.has_value());
		expect(eq(noimu->size(), 0));

		noimu = reader.getImuTimeRange(range.second + 1, range.second + 10000);
		expect(noimu.has_value());
		expect(eq(noimu->size(), 0));

		auto notriggers = reader.getTriggersTimeRange(-10000, -1000);
		expect(notriggers.has_value());
		expect(eq(notriggers->size(), 0));

		notriggers = reader.getTriggersTimeRange(range.first - 10000, range.first);
		expect(notriggers.has_value());
		expect(eq(notriggers->size(), 0));

		notriggers = reader.getTriggersTimeRange(range.second + 1, range.second + 10000);
		expect(notriggers.has_value());
		expect(eq(notriggers->size(), 0));

		expect(throws([&reader] {
			const auto _ = reader.getEventsTimeRange(10000, 1000);
		}));
		expect(throws([&reader] {
			const auto _ = reader.getTriggersTimeRange(10000, 1000);
		}));
		expect(throws([&reader] {
			const auto _ = reader.getImuTimeRange(10000, 1000);
		}));
	};

	"read_imu_range"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isImuStreamAvailable());
		const auto range = reader.getTimeRange();

		auto allImu = reader.getImuTimeRange(range.first, range.second + 1);

		// The available data should be this size in the sample file
		expect(eq(allImu->size(), 2188));

		// Get the time in the median
		int64_t sampleTime = allImu->at((allImu->size() / 5) + 5).timestamp;

		// Read from that time
		auto halfImu = reader.getImuTimeRange(sampleTime, allImu->back().timestamp);

		// It should still be within read time
		expect(eq(halfImu->front().timestamp, sampleTime));
		expect(eq(halfImu->back().timestamp, std::next(allImu->crbegin())->timestamp));

		auto allImu2 = reader.getImuTimeRange(allImu->front().timestamp, allImu->back().timestamp + 1);
		expect(ge(allImu2->front().timestamp, allImu->front().timestamp));
		expect(ge(allImu2->back().timestamp, allImu->back().timestamp));
	};

	"read_triggers_range"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isTriggerStreamAvailable());
		const auto range = reader.getTimeRange();

		auto allTriggers = reader.getTriggersTimeRange(range.first, range.second);

		expect(allTriggers->empty());
	};

	"read_pose_range"_test = [] {
		std::string filename = "./pose-test-file.aedat4";
		{
			dv::io::MonoCameraWriter::Config config("test");
			config.addStream<dv::Pose>("pose");
			dv::io::MonoCameraWriter writer(filename, config);

			writer.writePacket(
				dv::Pose(10, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
			writer.writePacket(dv::Pose(12, dv::Vec3f(0.1f, 0.2f, 0.3f),
								   dv::Quaternion(0.525322f, 0.f, 0.8509035f, 0.f), "other", "another"),
				"pose");
			writer.writePacket(
				dv::Pose(14, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
			writer.writePacket(
				dv::Pose(16, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
			writer.writePacket(
				dv::Pose(18, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
			writer.writePacket(
				dv::Pose(19, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
			writer.writePacket(
				dv::Pose(20, dv::Vec3f(0.1f, 0.2f, 0.3f), dv::Quaternion(1.0f, 0.f, 0.f, 0.f), "source", "target"),
				"pose");
		}
		dv::io::MonoCameraRecording reader(filename);

		expect(reader.isStreamAvailable("pose"));

		auto pose = reader.getNextStreamPacket<dv::Pose>("pose");
		expect(pose.has_value());
		expect(eq(pose->timestamp, 10));

		const auto range = reader.getTimeRange();
		expect(eq(range.first, 10));
		expect(eq(range.second, 20));

		auto poses = reader.getStreamTimeRange<dv::Pose>(12, 17, "pose");
		expect(eq(poses->size(), 3));
		expect(eq(poses->at(0).timestamp, 12));
		expect(eq(poses->at(1).timestamp, 14));
		expect(eq(poses->at(2).timestamp, 16));

		auto poses2 = reader.getStreamTimeRange<dv::Pose>(13, 15, "pose");
		expect(eq(poses2->size(), 1));
		expect(eq(poses2->at(0).timestamp, 14));

		auto poses3 = reader.getStreamTimeRange<dv::Pose>(21, 22, "pose");
		expect(poses3.has_value());
		expect(poses3->empty());

		auto poses4 = reader.getStreamTimeRange<dv::Pose>(14, 16, "pose");
		expect(eq(poses4->size(), 1));
		expect(eq(poses4->at(0).timestamp, 14));

		auto poses5 = reader.getStreamTimeRange<dv::Pose>(9, 11, "pose");
		expect(eq(poses5->size(), 1));
		expect(eq(poses5->at(0).timestamp, 10));

		auto poses6 = reader.getStreamTimeRange<dv::Pose>(16, 19, "pose");
		expect(eq(poses6->size(), 2));
		expect(eq(poses6->at(0).timestamp, 16));
		expect(eq(poses6->at(1).timestamp, 18));
	};

	"read_events_range"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		expect(reader.isEventStreamAvailable());
		const auto range = reader.getTimeRange();

		auto allEvent = reader.getEventsTimeRange(range.first, range.second + 1);

		expect(allEvent.has_value());
		expect(eq(allEvent->size(), 255283));
		auto sampleTime1 = allEvent->at(allEvent->size() / 3).timestamp();
		auto sampleTime2 = allEvent->at((allEvent->size() / 3) * 2).timestamp();

		auto events = reader.getEventsTimeRange(sampleTime1, sampleTime2);
		expect(eq(events->getLowestTime(), sampleTime1));
		expect(gt(events->getHighestTime(), sampleTime1));
		expect(lt(events->getHighestTime(), sampleTime2));

		auto events2 = reader.getEventsTimeRange(sampleTime1, sampleTime2 + 1);
		expect(eq(events2->getLowestTime(), sampleTime1));
		expect(eq(events2->getHighestTime(), sampleTime2));

		auto testE = allEvent->sliceTime(allEvent->getHighestTime(), allEvent->getHighestTime() + 1);
		expect(gt(testE.size(), 0));

		auto events3 = reader.getEventsTimeRange(allEvent->getHighestTime(), allEvent->getHighestTime() + 1);
		expect(gt(events3->size(), 0));
	};

	"read_trigger_range"_test = [] {
		std::string filename = "./trigger-test-file.aedat4";
		{
			dv::io::MonoCameraWriter::Config config("test");
			config.addTriggerStream();
			dv::io::MonoCameraWriter writer(filename, config);
			writer.setPackagingCount(3);

			writer.writeTrigger(dv::Trigger(10'000'000, dv::TriggerType::APS_FRAME_START));
			writer.writeTrigger(dv::Trigger(12'000'000, dv::TriggerType::APS_FRAME_START));
			writer.writeTrigger(dv::Trigger(14'000'000, dv::TriggerType::APS_FRAME_END));
			writer.writeTrigger(dv::Trigger(15'000'000, dv::TriggerType::APS_EXPOSURE_START));
			writer.writeTrigger(dv::Trigger(16'000'000, dv::TriggerType::APS_FRAME_START));
			writer.writeTrigger(dv::Trigger(17'000'000, dv::TriggerType::APS_EXPOSURE_END));
			writer.writeTrigger(dv::Trigger(18'000'000, dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE));
			writer.writeTrigger(dv::Trigger(20'000'000, dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE));
		}
		dv::io::MonoCameraRecording reader(filename);

		expect(reader.isTriggerStreamAvailable());
		const auto range = reader.getTimeRange();
		expect(eq(range.first, 10'000'000));
		expect(eq(range.second, 20'000'000));

		auto triggers = reader.getTriggersTimeRange(12'000'000, 16'000'000);
		expect(triggers.has_value());
		expect(eq(triggers->size(), 3));
		expect(eq(triggers->at(0).timestamp, 12'000'000));
		expect(eq(triggers->at(1).timestamp, 14'000'000));
		expect(eq(triggers->at(2).timestamp, 15'000'000));

		auto triggers2 = reader.getTriggersTimeRange(17'000'000, 19'999'999);
		expect(eq(triggers2->size(), 2));
		expect(eq(triggers2->at(0).timestamp, 17'000'000));
		expect(eq(triggers2->at(1).timestamp, 18'000'000));

		auto triggers3 = reader.getTriggersTimeRange(21'000'000, 22'000'000);
		expect(triggers3.has_value());
		expect(triggers3->empty());

		auto triggers4 = reader.getTriggersTimeRange(10'000'000, 20'000'000);
		expect(eq(triggers4->size(), 7));
		expect(eq(triggers4->at(0).timestamp, 10'000'000));
		expect(eq(triggers4->at(1).timestamp, 12'000'000));

		auto triggers5 = reader.getTriggersTimeRange(10'000'000, 20'000'001);
		expect(eq(triggers5->size(), 8));
		expect(eq(triggers5->front().timestamp, 10'000'000));
		expect(eq(triggers5->back().timestamp, 20'000'000));
	};

	"read_frame_range"_test = [] {
		std::string filename = "./frame-test-file.aedat4";
		{
			const cv::Size resolution(100, 100);

			dv::io::MonoCameraWriter::Config config("test");
			config.addFrameStream(resolution);
			dv::io::MonoCameraWriter writer(filename, config);

			expect(eq(config.findStreamResolution("frames").value(), resolution));

			writer.writeFrame(dv::Frame(10'000'000, cv::Mat::zeros(resolution, CV_8UC3)));
			writer.writeFrame(dv::Frame(12'000'000, cv::Mat::ones(resolution, CV_8UC3)));
			writer.writeFrame(dv::Frame(13'000'000, cv::Mat::zeros(resolution, CV_8UC3)));
			writer.writeFrame(dv::Frame(14'000'000, cv::Mat::zeros(resolution, CV_8UC3)));
		}

		dv::io::MonoCameraRecording reader(filename);

		expect(reader.isFrameStreamAvailable());
		const auto range = reader.getTimeRange();
		expect(eq(range.first, 10'000'000));
		expect(eq(range.second, 14'000'000));

		auto frames = reader.getFramesTimeRange(12'000'000, 14'000'000);
		expect(eq(frames->size(), 2));
		expect(eq(frames->at(0).timestamp, 12'000'000));
		expect(eq(frames->at(1).timestamp, 13'000'000));
		auto frames2 = reader.getFramesTimeRange(12'000'000, 12'000'000);
		expect(frames2->empty());

		auto frames3 = reader.getFramesTimeRange(12'000'000, 12'000'001);
		expect(eq(frames3->size(), 1));
		expect(eq(frames3->at(0).timestamp, 12'000'000));
	};

	"camera_naming"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		const auto &camera = reader.getCameraName();

		dv::io::MonoCameraRecording namedReader(file, camera);
		expect(eq(namedReader.getCameraName(), camera));

		expect(throws([&file] {
			dv::io::MonoCameraRecording namedReader(file, "fake camera name");
		}));
	};

	"resolution_lookup"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		auto resolution = reader.getEventResolution();
		expect(resolution.has_value());

		expect(eq(resolution->width, 640));
		expect(eq(resolution->height, 480));

		auto frameResolution = reader.getFrameResolution();
		expect(!frameResolution.has_value());
	};

	"file_introspection"_test = [&file] {
		dv::io::MonoCameraRecording reader(file);

		std::vector<std::string> streams = reader.getStreamNames();
		expect(eq(streams.size(), 3));

		expect(eq(std::count(streams.begin(), streams.end(), "events"), 1));
		expect(eq(std::count(streams.begin(), streams.end(), "triggers"), 1));
		expect(eq(std::count(streams.begin(), streams.end(), "imu"), 1));

		expect(reader.isStreamOfDataType<dv::EventPacket>("events"));
		expect(reader.isStreamOfDataType<dv::EventStore>("events"));
		expect(!reader.isStreamOfDataType<dv::IMUPacket>("events"));
		expect(!reader.isStreamOfDataType<dv::Frame>("events"));
		expect(!reader.isStreamOfDataType<dv::TriggerPacket>("events"));

		expect(!reader.isStreamOfDataType<dv::EventPacket>("triggers"));
		expect(!reader.isStreamOfDataType<dv::EventStore>("triggers"));
		expect(!reader.isStreamOfDataType<dv::IMUPacket>("triggers"));
		expect(!reader.isStreamOfDataType<dv::Frame>("triggers"));
		expect(reader.isStreamOfDataType<dv::TriggerPacket>("triggers"));

		expect(!reader.isStreamOfDataType<dv::EventPacket>("imu"));
		expect(!reader.isStreamOfDataType<dv::EventStore>("imu"));
		expect(reader.isStreamOfDataType<dv::IMUPacket>("imu"));
		expect(!reader.isStreamOfDataType<dv::Frame>("imu"));
		expect(!reader.isStreamOfDataType<dv::TriggerPacket>("imu"));
	};

	"no_error_on_empty_file"_test = [] {
		expect(nothrow([] {
			dv::io::MonoCameraRecording reader("./test_files/empty_stereo.aedat4");
			expect(!reader.getStreamNames().empty());
		}));
	};

	return EXIT_SUCCESS;
}

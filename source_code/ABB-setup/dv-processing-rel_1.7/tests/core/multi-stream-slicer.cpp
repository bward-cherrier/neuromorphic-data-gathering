#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/core/multi_stream_slicer.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;
	using namespace std::chrono_literals;

	"basic_slicing"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::EventStore> slicer("events");

		// You don't need to specify secondary stream type
		slicer.addStream<dv::TriggerPacket>("triggers");
		slicer.addStream<dv::IMUPacket>("imu");
		slicer.addStream<dv::cvector<dv::Frame>>("frames");

		slicer.doEveryTimeInterval(100ms, [](const auto &data) {
			// Retrieve events

			const auto events = data.template get<dv::EventStore>("events");
			expect(le(events.duration().count(), 100'000));

			const auto triggers = data.template get<dv::TriggerPacket>("triggers");
			expect(eq(triggers.elements.size(), events.size()));
			expect(eq(triggers.elements.front().timestamp, events.getLowestTime()));
			expect(eq(triggers.elements.back().timestamp, events.getHighestTime()));

			const auto imus = data.template get<dv::IMUPacket>("imu");
			expect(eq(imus.elements.size(), events.size()));
			expect(eq(imus.elements.front().timestamp, events.getLowestTime()));
			expect(eq(imus.elements.back().timestamp, events.getHighestTime()));

			const auto frames = data.template get<dv::cvector<dv::Frame>>("frames");
			expect(eq(frames.size(), events.size()));
			expect(eq(frames.front().timestamp, events.getLowestTime()));
			expect(eq(frames.back().timestamp, events.getHighestTime()));
		});

		size_t eventAmount         = 10000;
		int64_t timestampIncrement = 10000LL;
		int64_t timestamp          = 1000LL;

		dv::EventStore events;
		dv::TriggerPacket triggers;
		dv::IMUPacket imus;
		dv::cvector<dv::Frame> frames;
		dv::IMU imu;
		for (size_t i = 0; i < eventAmount; i++) {
			timestamp += timestampIncrement;
			events.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			triggers.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
			imu.timestamp = timestamp;
			imus.elements.push_back(imu);
			frames.emplace_back(timestamp, cv::Mat());
		}

		slicer.accept("events", events);
		slicer.accept("triggers", triggers);
		slicer.accept("imu", imus);
		slicer.accept("frames", frames);
	};

	"sparse_input_slicing"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::cvector<dv::Frame>> slicer("frames");

		size_t counter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto frames = std::get<dv::cvector<dv::Frame>>(data.at("frames"));
			counter           += frames.size();
		});

		size_t count               = 100;
		int64_t timestampIncrement = 100000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			frames.emplace_back(timestamp, cv::Mat());
		}

		slicer.accept("frames", frames);

		expect(eq(counter, count));
	};

	"additional_type"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::cvector<dv::Frame>, dv::TimedKeyPointPacket> slicer("frames");

		slicer.addStream<dv::TimedKeyPointPacket>("keypoints");

		size_t counter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto frames = std::get<dv::cvector<dv::Frame>>(data.at("frames"));
			counter           += frames.size();

			expect(eq(frames.size(), 10));

			const auto keypoints = std::get<dv::TimedKeyPointPacket>(data.at("keypoints"));
			expect(eq(keypoints.elements.size(), frames.size()));
			expect(eq(keypoints.elements.front().timestamp, frames.front().timestamp));
			expect(eq(keypoints.elements.back().timestamp, frames.back().timestamp));
			expect(eq(keypoints.elements.size(), 10));
		});

		size_t count               = 1000;
		int64_t timestampIncrement = 1000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		dv::TimedKeyPointPacket keypoints;
		dv::TimedKeyPoint kpt;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			frames.emplace_back(timestamp, cv::Mat());
			kpt.timestamp = timestamp;
			keypoints.elements.push_back(kpt);
		}

		expect(eq(counter, 0));
		slicer.accept("keypoints", keypoints);
		expect(eq(counter, 0));
		slicer.accept("frames", frames);

		expect(eq(counter, count));
	};

	"slice_by_number_backward"_test = [] {
		dv::MultiStreamSlicer<dv::EventStore> slicer("events_main");
		slicer.addStream<dv::EventStore>("events_secondary");

		const size_t count               = 1000;
		const int64_t timestampIncrement = 1000LL;
		int64_t timestamp                = 1000LL;
		dv::EventStore events;
		for (size_t i = 0; i < count; i++) {
			timestamp += timestampIncrement;
			events.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}

		size_t backwardCounter = 0;
		size_t perSliceNumber  = 10;
		slicer.doEveryNumberOfElements(
			perSliceNumber,
			[&backwardCounter](const auto &data) {
				const dv::EventStore mainEvents      = std::get<dv::EventStore>(data.at("events_main"));
				const dv::EventStore secondaryEvents = std::get<dv::EventStore>(data.at("events_secondary"));
				expect(eq(mainEvents.size(), secondaryEvents.size()));
				expect(eq(mainEvents.getHighestTime(), secondaryEvents.getHighestTime()));
				expect(eq(mainEvents.getLowestTime(), secondaryEvents.getLowestTime()));
				backwardCounter += mainEvents.size();
			},
			dv::TimeSlicingApproach::BACKWARD);

		size_t forwardCounter = 0;
		slicer.doEveryNumberOfElements(
			perSliceNumber,
			[&forwardCounter](const auto &data) {
				const dv::EventStore mainEvents      = std::get<dv::EventStore>(data.at("events_main"));
				const dv::EventStore secondaryEvents = std::get<dv::EventStore>(data.at("events_secondary"));
				forwardCounter += (mainEvents.size() == secondaryEvents.size()) ? mainEvents.size() : 0;
				expect(eq(mainEvents.size(), secondaryEvents.size()));
				expect(eq(mainEvents.getHighestTime(), secondaryEvents.getHighestTime()));
				expect(eq(mainEvents.getLowestTime(), secondaryEvents.getLowestTime()));
			},
			dv::TimeSlicingApproach::FORWARD);

		slicer.accept("events_main", events);
		slicer.accept("events_secondary", events);

		expect(eq(backwardCounter, count));
		expect(eq(forwardCounter, count - perSliceNumber));
	};

	"slice_one_by_one"_test = [] {
		dv::MultiStreamSlicer<dv::EventStore> slicer("events_main");
		slicer.addStream<dv::EventStore>("events_secondary");

		const size_t count               = 1000;
		const int64_t timestampIncrement = 1000LL;
		int64_t timestamp                = 1000LL;
		dv::EventStore eventsMain;
		dv::EventStore eventsSecondary;
		eventsSecondary.emplace_back(
			timestamp - (timestampIncrement / 2), static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		for (size_t i = 0; i < count; i++) {
			timestamp += timestampIncrement;
			eventsMain.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			eventsSecondary.emplace_back(
				timestamp + (timestampIncrement / 2), static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}

		size_t numberOfEventsProcessedBackward = 0, numberOfEventsProcessedForward = 0;
		slicer.doEveryNumberOfElements(
			1,
			[&numberOfEventsProcessedBackward](const auto &data) {
				const dv::EventStore mainEvents      = std::get<dv::EventStore>(data.at("events_main"));
				const dv::EventStore secondaryEvents = std::get<dv::EventStore>(data.at("events_secondary"));

				expect(eq(mainEvents.size(), secondaryEvents.size()));
				expect(eq(mainEvents.size(), 1));
				expect(gt(mainEvents.at(0).timestamp(), secondaryEvents.at(0).timestamp()));
				++numberOfEventsProcessedBackward;
			},
			dv::TimeSlicingApproach::BACKWARD);

		slicer.doEveryNumberOfElements(
			1,
			[&numberOfEventsProcessedForward](const auto &data) {
				const dv::EventStore mainEvents      = std::get<dv::EventStore>(data.at("events_main"));
				const dv::EventStore secondaryEvents = std::get<dv::EventStore>(data.at("events_secondary"));

				expect(eq(mainEvents.size(), secondaryEvents.size()));
				expect(eq(mainEvents.size(), 1));
				expect(lt(mainEvents.at(0).timestamp(), secondaryEvents.at(0).timestamp()));
				++numberOfEventsProcessedForward;
			},
			dv::TimeSlicingApproach::FORWARD);

		slicer.accept("events_main", eventsMain);
		slicer.accept("events_secondary", eventsSecondary);
		expect(eq(numberOfEventsProcessedBackward, eventsMain.size()));
		expect(eq(numberOfEventsProcessedForward, eventsMain.size() - 1));
	};

	"slice_one_by_one_lagging_secondary_stream"_test = [] {
		dv::MultiStreamSlicer<dv::cvector<dv::Trigger>> slicer("triggers");
		slicer.addStream<dv::EventStore>("events");

		size_t numberOfEventsPerTrigger  = 1000;
		const size_t count               = 1000;
		const int64_t timestampIncrement = 1000LL;
		int64_t timestamp                = 1000LL;

		size_t numberOfTriggersProcessedBackward = 0, numberOfTriggersProcessedForward = 0;
		slicer.doEveryNumberOfElements(
			1,
			[&numberOfTriggersProcessedBackward, &numberOfEventsPerTrigger](const auto &data) {
				const auto events   = std::get<dv::EventStore>(data.at("events"));
				const auto triggers = std::get<dv::cvector<dv::Trigger>>(data.at("triggers"));

				expect(eq(triggers.size(), 1));
				expect(eq(events.size(), numberOfEventsPerTrigger));
				++numberOfTriggersProcessedBackward;
			},
			dv::TimeSlicingApproach::BACKWARD);

		slicer.doEveryNumberOfElements(
			1,
			[&numberOfTriggersProcessedForward, &numberOfEventsPerTrigger](const auto &data) {
				const auto events   = std::get<dv::EventStore>(data.at("events"));
				const auto triggers = std::get<dv::cvector<dv::Trigger>>(data.at("triggers"));

				expect(eq(triggers.size(), 1));
				expect(eq(events.size(), numberOfEventsPerTrigger));
				++numberOfTriggersProcessedForward;
			},
			dv::TimeSlicingApproach::FORWARD);

		dv::EventStore events;
		dv::cvector<dv::Trigger> triggers;
		for (size_t i = 0; i < count; i++) {
			for (size_t j = 0; j < numberOfEventsPerTrigger; ++j) {
				events.emplace_back(
					timestamp + (timestampIncrement / 2), static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			}
			timestamp += timestampIncrement;
			triggers.emplace_back(timestamp, dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE);
		}
		timestamp += timestampIncrement;

		slicer.accept("events", events);
		slicer.accept("triggers", triggers);

		// Note that as events always lag behind triggers, the last trigger will not be processed until enough events
		//  are provided
		expect(eq(numberOfTriggersProcessedBackward, triggers.size() - 1));
		expect(eq(numberOfTriggersProcessedForward, triggers.size() - 2));

		// To manually force the slicer to process the last trigger, artificially advance the event stream
		slicer.setStreamSeekTime("events", timestamp);
		expect(eq(numberOfTriggersProcessedBackward, triggers.size()));
		expect(eq(numberOfTriggersProcessedForward, triggers.size() - 1));
	};

	"modify_number_job"_test = [] {
		dv::MultiStreamSlicer<dv::TriggerPacket> slicer("main");
		slicer.addStream<dv::TriggerPacket>("secondary");

		const size_t count               = 1000;
		const int64_t timestampIncrement = 1000LL;
		int64_t timestamp                = 1000LL;
		dv::TriggerPacket triggers;
		for (size_t i = 0; i < count; i++) {
			timestamp += timestampIncrement;
			triggers.elements.emplace_back(timestamp, dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE);
		}

		size_t numberCounter  = 0;
		size_t perSliceNumber = 10;
		const int32_t numberJobId
			= slicer.doEveryNumberOfElements(perSliceNumber, [&numberCounter, &perSliceNumber](const auto &data) {
				  const auto mainData      = std::get<dv::TriggerPacket>(data.at("main"));
				  const auto secondaryData = std::get<dv::TriggerPacket>(data.at("secondary"));
				  expect(eq(mainData.elements.size(), perSliceNumber));
				  expect(eq(mainData.elements.size(), secondaryData.elements.size()));
				  expect(eq(mainData.elements.back().timestamp, secondaryData.elements.back().timestamp));
				  expect(eq(mainData.elements.front().timestamp, secondaryData.elements.front().timestamp));
				  numberCounter += mainData.elements.size();
			  });

		size_t timeCounter = 0;
		slicer.doEveryTimeInterval(10ms, [&timeCounter](const auto &data) {
			const auto mainData      = std::get<dv::TriggerPacket>(data.at("main"));
			const auto secondaryData = std::get<dv::TriggerPacket>(data.at("secondary"));
			timeCounter              += mainData.elements.size();
			expect(eq(mainData.elements.size(), secondaryData.elements.size()));
			expect(eq(mainData.elements.back().timestamp, secondaryData.elements.back().timestamp));
			expect(eq(mainData.elements.front().timestamp, secondaryData.elements.front().timestamp));
		});

		dv::TriggerPacket packet;
		packet.elements = dv::cvector<dv::Trigger>(triggers.elements.begin(), triggers.elements.begin() + 500);

		slicer.accept("main", packet);
		slicer.accept("secondary", packet);

		expect(eq(numberCounter, 500));
		// One less because time interval does not execute while data in the future was not received
		expect(eq(timeCounter, 490));

		perSliceNumber = 100;
		slicer.modifyNumberInterval(numberJobId, perSliceNumber);

		packet.elements = dv::cvector<dv::Trigger>(triggers.elements.begin() + 500, triggers.elements.end());

		slicer.accept("main", packet);
		slicer.accept("secondary", packet);

		expect(eq(numberCounter, 1000));
		expect(eq(timeCounter, 990));
	};

	"modify_job"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::EventStore> slicer("events");

		slicer.addStream<dv::TriggerPacket>("triggers");
		slicer.addStream<dv::cvector<dv::Frame>>("frames");

		size_t eventsCounter   = 0;
		size_t triggersCounter = 0;
		size_t framesCounter   = 0;
		size_t sliceSize       = 10;
		bool callbackCalled    = false;

		int jobId = slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			callbackCalled = true;

			const auto events = data.template get<dv::EventStore>("events");
			eventsCounter     += events.size();
			expect(eq(events.size(), sliceSize));

			const auto triggers = data.template get<dv::TriggerPacket>("triggers");
			expect(eq(triggers.elements.size(), sliceSize));
			triggersCounter += triggers.elements.size();

			const auto frames = data.template get<dv::cvector<dv::Frame>>("frames");
			expect(eq(frames.size(), sliceSize));
			framesCounter += frames.size();
		});

		size_t count               = 1000;
		int64_t timestampIncrement = 1000LL;
		int64_t timestamp          = 1000LL;
		dv::EventStore events;
		dv::TriggerPacket triggers;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			events.emplace_back(timestamp, 0, 0, false);
			triggers.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
			frames.emplace_back(timestamp, cv::Mat());
		}

		slicer.accept("events", events);
		slicer.accept("triggers", triggers);
		slicer.accept("frames", frames);
		expect(eq(eventsCounter, count));
		expect(eq(triggersCounter, count));
		expect(eq(framesCounter, count));
		expect(callbackCalled);
		callbackCalled = false;

		slicer.modifyTimeInterval(jobId, 100ms);
		sliceSize = 100;
		events    = dv::EventStore();
		triggers.elements.clear();
		frames.clear();
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			events.emplace_back(timestamp, 0, 0, false);
			triggers.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
			frames.emplace_back(timestamp, cv::Mat());
		}

		slicer.accept("triggers", triggers);
		slicer.accept("events", events);
		slicer.accept("frames", frames);

		expect(eq(eventsCounter, count * 2));
		expect(eq(triggersCounter, count * 2));
		expect(eq(framesCounter, count * 2));
		expect(callbackCalled);
		callbackCalled = false;

		slicer.modifyTimeInterval(jobId, 200ms);
		sliceSize = 200;
		events    = dv::EventStore();
		triggers.elements.clear();
		frames.clear();

		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			events.emplace_back(timestamp, 0, 0, false);
			triggers.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
			frames.emplace_back(timestamp, cv::Mat());
		}

		slicer.accept("events", events);
		slicer.accept("triggers", triggers);
		slicer.accept("frames", frames);

		expect(eq(eventsCounter, count * 3));
		expect(eq(triggersCounter, count * 3));
		expect(eq(framesCounter, count * 3));
		expect(callbackCalled);
	};

	"pass_frame_element"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::cvector<dv::Frame>> slicer("frames");

		size_t counter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto frames = std::get<dv::cvector<dv::Frame>>(data.at("frames"));
			counter           += frames.size();
		});

		size_t count               = 100;
		int64_t timestampIncrement = 100000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			slicer.accept("frames", dv::Frame(timestamp, cv::Mat()));
		}
		expect(eq(counter, count));
	};

	"pass_trigger_element"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::TriggerPacket> slicer("triggers");

		size_t counter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto triggers = std::get<dv::TriggerPacket>(data.at("triggers"));
			counter             += triggers.elements.size();
		});

		size_t count               = 100;
		int64_t timestampIncrement = 100000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			slicer.accept("triggers", dv::Trigger(timestamp, dv::TriggerType::APS_EXPOSURE_END));
		}
		expect(eq(counter, count));
	};

	"pass_events_element"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::EventStore> slicer("events");
		slicer.addStream<dv::TriggerPacket>("triggers");

		size_t counter        = 0;
		size_t triggerCounter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto events = std::get<dv::EventStore>(data.at("events"));
			counter           += events.size();

			const auto triggers = std::get<dv::TriggerPacket>(data.at("triggers"));
			triggerCounter      += triggers.elements.size();
		});

		size_t count               = 100;
		int64_t timestampIncrement = 100000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			slicer.accept("events", dv::Event(timestamp, 0, 0, false));
			slicer.accept("triggers", dv::Trigger(timestamp, dv::TriggerType::APS_EXPOSURE_END));
		}
		expect(eq(counter, count));
		expect(eq(triggerCounter, count));
	};

	"wrong_types"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::EventStore, dv::cvector<dv::Event>> slicer("events");

		size_t counter = 0;
		slicer.doEveryTimeInterval(10ms, [&](const auto &data) {
			const auto events = data.template get<dv::EventStore>("events");
			counter           += events.size();

			expect(throws([data] {
				// Wrong type
				data.template get<dv::cvector<dv::Event>>("events");
			}));
			expect(throws([data] {
				// Wrong type
				data.template get<dv::IMUPacket>("events");
			}));
			expect(throws([data] {
				// Wrong type
				data.template get<dv::cvector<dv::Frame>>("events");
			}));
			expect(throws([data] {
				// Wrong stream name
				data.template get<dv::EventStore>("frames");
			}));
		});

		size_t count               = 100;
		int64_t timestampIncrement = 100000LL;
		int64_t timestamp          = 1000LL;
		dv::cvector<dv::Frame> frames;
		for (size_t i = 0; i <= count; i++) {
			timestamp += timestampIncrement;
			slicer.accept("events", dv::Event(timestamp, 0, 0, false));
		}
		expect(eq(counter, count));
	};

	"number_slice_within_microsecond"_test = [] {
		// Initialize with main stream type and main stream name
		dv::MultiStreamSlicer<dv::EventStore> slicer("main");
		slicer.addStream<dv::EventStore>("secondary");

		bool first = true;

		slicer.doEveryNumberOfElements(10, [&](const auto &data) {
			const auto events    = data.template get<dv::EventStore>("main");
			const auto secondary = data.template get<dv::EventStore>("secondary");
			expect(eq(events.size(), 10));
			if (first) {
				expect(eq(secondary.size(), 1000));
				first = false;
			}
			else {
				expect(eq(secondary.size(), 0));
			}
		});

		const size_t count = 1000;

		const int64_t timestamp = 1000LL;
		dv::EventStore events;
		for (size_t i = 0; i < count; i++) {
			events.emplace_back(timestamp, 0, 0, false);
		}

		expect(nothrow([&slicer, &events] {
			slicer.accept("main", events);
			slicer.accept("secondary", events);
		}));
	};

	return 0;
}

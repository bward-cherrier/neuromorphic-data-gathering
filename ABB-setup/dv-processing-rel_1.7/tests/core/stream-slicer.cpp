#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/core/stream_slicer.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;
	using namespace std::chrono_literals;

	"event_store_slicer"_test = [] {
		dv::StreamSlicer<dv::EventStore> slicer;

		size_t timeCounter  = 0;
		size_t rangeCounter = 0;
		int rangeJobId, timeJobId;

		size_t eventAmount         = 10000;
		int64_t timestampIncrement = 10000LL;
		int64_t timestamp          = 1000LL;

		rangeJobId = slicer.doEveryNumberOfElements(100, [&](const dv::EventStore &data) {
			rangeCounter += data.size();
		});

		timeJobId = slicer.doEveryTimeInterval(100ms, [&](const dv::EventStore &data) {
			timeCounter += data.size();
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));

		static_assert(dv::concepts::TimestampedByAccessor<dv::Event>);
		static_assert(dv::concepts::TimestampedByMember<dv::Event> || dv::concepts::TimestampedByAccessor<dv::Event>);

		dv::EventStore data1;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data1.emplace_back(timestamp, 0, 0, false);
		}
		slicer.accept(std::move(data1));

		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));

		rangeCounter = 0;
		timeCounter  = 0;
		dv::EventStore data2;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data2.emplace_back(timestamp, 0, 0, false);
		}
		slicer.modifyNumberInterval(rangeJobId, 200);
		slicer.modifyTimeInterval(timeJobId, dv::Duration{200000});

		slicer.accept(std::move(data2));
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
		expect(throws([&] {
			slicer.modifyNumberInterval(timeJobId, 200);
		}));
		expect(throws([&] {
			slicer.modifyTimeInterval(rangeJobId, dv::Duration{200000});
		}));

		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	"event_vector_slicer"_test = [] {
		dv::StreamSlicer<std::vector<dv::Event>> slicer;

		size_t timeCounter  = 0;
		size_t rangeCounter = 0;
		int rangeJobId, timeJobId;

		size_t eventAmount         = 10000;
		int64_t timestampIncrement = 10000LL;
		int64_t timestamp          = 1000LL;

		rangeJobId = slicer.doEveryNumberOfElements(100, [&](const std::vector<dv::Event> &data) {
			rangeCounter += data.size();
		});

		timeJobId = slicer.doEveryTimeInterval(100ms, [&](const std::vector<dv::Event> &data) {
			timeCounter += data.size();
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));

		static_assert(dv::concepts::TimestampedByAccessor<dv::Event>);
		static_assert(dv::concepts::TimestampedByMember<dv::Event> || dv::concepts::TimestampedByAccessor<dv::Event>);

		std::vector<dv::Event> data1;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data1.emplace_back(timestamp, 0, 0, false);
		}
		slicer.accept(std::move(data1));

		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));

		rangeCounter = 0;
		timeCounter  = 0;
		std::vector<dv::Event> data2;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data2.emplace_back(timestamp, 0, 0, false);
		}
		slicer.modifyNumberInterval(rangeJobId, 200);
		slicer.modifyTimeInterval(timeJobId, dv::Duration{200000});

		slicer.accept(std::move(data2));
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
		expect(throws([&] {
			slicer.modifyNumberInterval(timeJobId, 200);
		}));
		expect(throws([&] {
			slicer.modifyTimeInterval(rangeJobId, dv::Duration{200000});
		}));

		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	"trigger_packet_slicer"_test = [] {
		dv::StreamSlicer<dv::TriggerPacket> slicer;

		size_t timeCounter  = 0;
		size_t rangeCounter = 0;
		int rangeJobId, timeJobId;

		size_t eventAmount         = 10000;
		int64_t timestampIncrement = 10000LL;
		int64_t timestamp          = 1000LL;

		rangeJobId = slicer.doEveryNumberOfElements(100, [&](const dv::TriggerPacket &data) {
			rangeCounter += data.elements.size();
		});

		timeJobId = slicer.doEveryTimeInterval(100ms, [&](const dv::TriggerPacket &data) {
			timeCounter += data.elements.size();
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));

		static_assert(dv::concepts::TimestampedByAccessor<dv::Event>);
		static_assert(dv::concepts::TimestampedByMember<dv::Event> || dv::concepts::TimestampedByAccessor<dv::Event>);

		dv::TriggerPacket data1;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data1.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
		}
		slicer.accept(std::move(data1));

		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));

		rangeCounter = 0;
		timeCounter  = 0;
		dv::TriggerPacket data2;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data2.elements.emplace_back(timestamp, dv::TriggerType::APS_EXPOSURE_END);
		}
		slicer.modifyNumberInterval(rangeJobId, 200);
		slicer.modifyTimeInterval(timeJobId, dv::Duration{200000});

		slicer.accept(std::move(data2));
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
		expect(throws([&] {
			slicer.modifyNumberInterval(timeJobId, 200);
		}));
		expect(throws([&] {
			slicer.modifyTimeInterval(rangeJobId, dv::Duration{200000});
		}));

		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	"frame_slicer"_test = [] {
		dv::StreamSlicer<dv::cvector<dv::Frame>> slicer;

		size_t timeCounter  = 0;
		size_t rangeCounter = 0;
		int rangeJobId, timeJobId;

		size_t eventAmount         = 10000;
		int64_t timestampIncrement = 10000LL;
		int64_t timestamp          = 1000LL;

		rangeJobId = slicer.doEveryNumberOfElements(100, [&](const dv::cvector<dv::Frame> &data) {
			rangeCounter += data.size();
		});

		timeJobId = slicer.doEveryTimeInterval(100ms, [&](const dv::cvector<dv::Frame> &data) {
			timeCounter += data.size();
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));

		static_assert(dv::concepts::TimestampedByAccessor<dv::Event>);
		static_assert(dv::concepts::TimestampedByMember<dv::Event> || dv::concepts::TimestampedByAccessor<dv::Event>);

		dv::cvector<dv::Frame> data1;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data1.emplace_back(timestamp, cv::Mat());
		}
		slicer.accept(std::move(data1));

		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));

		rangeCounter = 0;
		timeCounter  = 0;
		dv::cvector<dv::Frame> data2;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			data2.emplace_back(timestamp, cv::Mat());
		}
		slicer.modifyNumberInterval(rangeJobId, 200);
		slicer.modifyTimeInterval(timeJobId, dv::Duration{200000});

		slicer.accept(std::move(data2));
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
		expect(throws([&] {
			slicer.modifyNumberInterval(timeJobId, 200);
		}));
		expect(throws([&] {
			slicer.modifyTimeInterval(rangeJobId, dv::Duration{200000});
		}));

		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	return 0;
}

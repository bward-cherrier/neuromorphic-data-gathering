#include "../../include/dv-processing/core/stereo_event_stream_slicer.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;
	using namespace std::chrono_literals;

	"add_jobs"_test = [] {
		dv::StereoEventStreamSlicer slicer;
		int rangeJobId = slicer.doEveryNumberOfEvents(100, [](const dv::EventStore &, const dv::EventStore &) {
		});

		int timeJobId = slicer.doEveryTimeInterval(100ms, [](const dv::EventStore &, const dv::EventStore &) {
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));
	};

	"run_jobs"_test = [] {
		dv::StereoEventStreamSlicer slicer;
		size_t leftRangeCounter  = 0;
		size_t rightRangeCounter = 0;
		size_t leftTimeCounter   = 0;
		size_t rightTimeCounter  = 0;
		slicer.doEveryNumberOfEvents(
			100, [&leftRangeCounter, &rightRangeCounter](const dv::EventStore &left, const dv::EventStore &right) {
				leftRangeCounter  += left.size();
				rightRangeCounter += right.size();
				expect(eq(left.size(), right.size()));
			});

		slicer.doEveryTimeInterval(
			100ms, [&leftTimeCounter, &rightTimeCounter](const dv::EventStore &left, const dv::EventStore &right) {
				leftTimeCounter  += left.size();
				rightTimeCounter += right.size();
				expect(eq(left.size(), right.size()));
			});

		int64_t timestamp = 1000LL;
		dv::EventStore events;
		size_t eventAmount = 10000;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += 10000LL;
			events.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}
		slicer.accept(events, events);

		expect(eq(leftRangeCounter, eventAmount));
		expect(eq(rightRangeCounter, eventAmount));
		expect(eq(leftTimeCounter, eventAmount));
		expect(eq(rightTimeCounter, eventAmount));
	};

	"remove_jobs"_test = [] {
		dv::StereoEventStreamSlicer slicer;
		int rangeJobId = slicer.doEveryNumberOfEvents(100, [](const dv::EventStore &, const dv::EventStore &) {
		});

		int timeJobId = slicer.doEveryTimeInterval(100ms, [](const dv::EventStore &, const dv::EventStore &) {
		});
		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));
		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	"stream_synchronization"_test = [] {
		dv::StereoEventStreamSlicer slicer;
		int64_t leftMaxTime   = -1;
		int64_t rightMaxTime  = -1;
		int64_t timeIncrement = 100LL;

		slicer.doEveryTimeInterval(10ms, [&leftMaxTime, &rightMaxTime](const auto &left, const auto &right) {
			leftMaxTime  = left.getHighestTime();
			rightMaxTime = right.getHighestTime();
			expect(eq(left.duration().count(), 9'900));
			expect(eq(right.duration().count(), 9'900));
		});

		int64_t timestamp = 1000LL;
		dv::EventStore events;
		size_t eventAmount = 10000;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timeIncrement;
			events.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}

		slicer.accept(events, dv::EventStore());

		// Since we haven't received anything on the right side, we ignore it
		expect(eq(leftMaxTime, -1LL));
		expect(eq(rightMaxTime, -1LL));

		// Now we separately put in the right, so we now expect the maxTimes are set and has the same value
		slicer.accept(dv::EventStore(), events);
		expect(neq(leftMaxTime, -1LL));
		expect(neq(rightMaxTime, -1LL));
		expect(eq(leftMaxTime, rightMaxTime));

		dv::EventStore newEvents;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timeIncrement;
			newEvents.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}
		leftMaxTime  = -1;
		rightMaxTime = -1;

		// Add more events, this should output the same thing as expected before
		slicer.accept(newEvents, newEvents);
		expect(neq(leftMaxTime, -1LL));
		expect(neq(rightMaxTime, -1LL));
		expect(eq(leftMaxTime, rightMaxTime));
	};

	"stream_with_offset"_test = [] {
		dv::StereoEventStreamSlicer slicer;
		int64_t leftMaxTime   = -1;
		int64_t rightMaxTime  = -1;
		int64_t timeIncrement = 100LL;

		slicer.doEveryTimeInterval(10ms, [&leftMaxTime, &rightMaxTime](const auto &left, const auto &right) {
			leftMaxTime  = left.getHighestTime();
			rightMaxTime = right.getHighestTime();
			expect(eq(left.duration().count(), 9'900));
			expect(eq(right.duration().count(), 9'900));
		});

		int64_t timestamp         = 100'000LL;
		size_t eventAmount        = 10'000;
		int64_t leftToRightOffset = 20'000LL;

		{
			dv::EventStore leftEvents, rightEvents;
			for (size_t i = 0; i <= eventAmount; i++) {
				timestamp += timeIncrement;
				leftEvents.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
				rightEvents.emplace_back(
					timestamp - leftToRightOffset, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			}
			slicer.accept(leftEvents, rightEvents);
		}
		{
			dv::EventStore leftEvents, rightEvents;
			for (size_t i = 0; i <= eventAmount; i++) {
				timestamp += timeIncrement;
				leftEvents.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
				rightEvents.emplace_back(
					timestamp - leftToRightOffset, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			}
			slicer.accept(leftEvents, rightEvents);
		}
	};
}

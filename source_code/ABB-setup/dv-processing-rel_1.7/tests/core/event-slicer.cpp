#include "../../include/dv-processing/data/generate.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	// Slicer
	dv::EventStreamSlicer slicer;

	size_t timeCounter  = 0;
	size_t rangeCounter = 0;
	int rangeJobId, timeJobId;

	size_t eventAmount         = 10000;
	int64_t timestampIncrement = 10000LL;
	int64_t timestamp          = 1000LL;

	"add_jobs"_test = [&] {
		rangeJobId = slicer.doEveryNumberOfElements(100, [&](const dv::EventStore &store) {
			rangeCounter += store.size();
		});

		timeJobId = slicer.doEveryTimeInterval(dv::Duration{100000}, [&](const dv::EventStore &store) {
			timeCounter += store.size();
		});

		expect(slicer.hasJob(rangeJobId));
		expect(slicer.hasJob(timeJobId));
	};

	"run_jobs"_test = [&] {
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			slicer.accept(dv::Event(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false));
		}
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
	};

	"run_modified_jobs"_test = [&] {
		rangeCounter = 0;
		timeCounter  = 0;
		dv::EventStore store;
		for (size_t i = 0; i <= eventAmount; i++) {
			timestamp += timestampIncrement;
			store.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		}
		slicer.modifyNumberInterval(rangeJobId, 200);
		slicer.modifyTimeInterval(timeJobId, dv::Duration{200000});

		slicer.accept(store);
		expect(eq(rangeCounter, eventAmount));
		expect(eq(timeCounter, eventAmount));
		expect(throws([&] {
			slicer.modifyNumberInterval(timeJobId, 200);
		}));
		expect(throws([&] {
			slicer.modifyTimeInterval(rangeJobId, dv::Duration{200000});
		}));
	};

	"remove_jobs"_test = [&] {
		slicer.removeJob(rangeJobId);
		expect(!slicer.hasJob(rangeJobId));
		slicer.removeJob(timeJobId);
		expect(!slicer.hasJob(timeJobId));
	};

	"count_events_in_slicer"_test = [] {
		dv::EventStreamSlicer streamSlicer;
		size_t counter = 0;
		int jobId      = streamSlicer.doEveryNumberOfElements(1000, [&counter](const dv::EventStore &events) {
            counter += events.size();
        });

		expect(streamSlicer.hasJob(jobId));
		streamSlicer.accept(dv::data::generate::uniformlyDistributedEvents(100, cv::Size(100, 100), 10'000, 0));
		expect(eq(counter, 10000));
	};
}

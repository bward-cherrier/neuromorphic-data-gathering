#include "../../include/dv-processing/core/event.hpp"
#include "../../include/dv-processing/data/generate.hpp"
#include "../../include/dv-processing/noise/background_activity_noise_filter.hpp"
#include "../../include/dv-processing/noise/fast_decay_noise_filter.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

template<class FilterClass>
void noiseTest() {
	cv::Size resolution(100, 100);

	FilterClass filter(resolution);

	dv::EventStore events = dv::data::generate::eventTestSet(100'000, resolution);
	filter.accept(events);
	auto filtered = filter.generateEvents();
	expect(gt(filtered.size(), 0));

	events = dv::data::generate::eventTestSet(150'000, resolution);
	filter.accept(events);
	filtered = filter.generateEvents();
	expect(gt(filtered.size(), 0));

	events = dv::data::generate::eventTestSet(200'000, resolution);
	filter.accept(events);
	filtered = filter.generateEvents();
	expect(gt(filtered.size(), 0));

	// By now we have initialized the event filter, let's add stray events and see whether they are filtered
	events = dv::data::generate::eventTestSet(250'000, resolution);

	auto roiInput = dv::boundingRect(events);

	events.emplace_back(251'000, 0, 0, true);
	events.emplace_back(251'000, 99, 99, true);
	events.emplace_back(251'000, 99, 0, true);
	events.emplace_back(251'000, 2, 99, true);
	filter.accept(events);
	filtered = filter.generateEvents();

	expect(gt(filtered.size(), 0));
	auto roiFiltered = dv::boundingRect(filtered);

	expect(eq(roiFiltered, roiInput));
}

int main() {
	"simple_noise"_test = [] {
		noiseTest<dv::noise::FastDecayNoiseFilter<>>();
		noiseTest<dv::noise::BackgroundActivityNoiseFilter<>>();
	};

	return EXIT_SUCCESS;
}

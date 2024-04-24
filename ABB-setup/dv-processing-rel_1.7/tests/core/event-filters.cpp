#include "../../include/dv-processing/core/event.hpp"
#include "../../include/dv-processing/core/filters.hpp"
#include "../../include/dv-processing/data/generate.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;
	using namespace std::chrono_literals;

	"roi_filter"_test = [] {
		cv::Rect roi(40, 40, 10, 10);
		auto events = dv::data::generate::eventTestSet(1000, cv::Size(100, 100));

		auto filter = dv::EventRegionFilter<>(roi);
		filter.accept(events);

		dv::EventStore filtered = filter.generateEvents();
		expect(eq(dv::boundingRect(filtered), roi));
	};

	"polarity_filter"_test = [] {
		auto events = dv::data::generate::eventTestSet(1000, cv::Size(100, 100));

		{
			auto filter = dv::EventPolarityFilter<>(true);
			filter.accept(events);
			dv::EventStore filtered = filter.generateEvents();

			expect(eq(filtered.size(), events.size()));
		}

		{
			auto filter = dv::EventPolarityFilter<>(false);
			filter.accept(events);
			dv::EventStore filtered = filter.generateEvents();

			expect(eq(filtered.size(), 0));
		}
	};

	"filter_chain"_test = [] {
		dv::EventFilterChain<> filterChain;
		cv::Rect roi(40, 40, 10, 10);

		filterChain.addFilter(std::make_unique<dv::EventPolarityFilter<>>(true));
		filterChain.addFilter(std::make_unique<dv::EventRegionFilter<>>(roi));

		auto events = dv::data::generate::eventTestSet(1000, cv::Size(100, 100));

		filterChain.accept(events);

		dv::EventStore filtered = filterChain.generateEvents();
		expect(eq(dv::boundingRect(filtered), roi));
	};

	"refractory_period_filter"_test = [] {
		cv::Size resolution(100, 100);

		// Refractory period of 1 millisecond
		dv::RefractoryPeriodFilter<> filter(resolution, 1ms);

		// Following events have a large temporal distance, so they are not filtered
		dv::EventStore events = dv::data::generate::eventLine(
			100'000, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1), 50);
		filter.accept(events);
		auto filtered = filter.generateEvents();
		expect(eq(filtered.size(), events.size()));

		events = dv::data::generate::eventLine(
			150'000, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1), 50);
		filter.accept(events);
		filtered = filter.generateEvents();
		expect(eq(filtered.size(), events.size()));

		events = dv::data::generate::eventLine(
			200'000, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1), 50);
		filter.accept(events);
		filtered = filter.generateEvents();
		expect(eq(filtered.size(), events.size()));

		// Let's add events with temporal distance less than configured refractory period
		events = dv::data::generate::eventLine(
			200'100, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1), 50);
		filter.accept(events);
		filtered = filter.generateEvents();
		// All events must be filtered out
		expect(eq(filtered.size(), 0ULL));

		// Reduce the refractory period, this should allow the events to pass
		filter.setRefractoryPeriod(20us);

		events = dv::data::generate::eventLine(
			200'200, cv::Point(0, 0), cv::Point(resolution.width - 1, resolution.height - 1), 50);
		filter.accept(events);
		filtered = filter.generateEvents();
		// All events must be filtered out
		expect(eq(filtered.size(), events.size()));

		// Just a smoke test for the getter method
		expect(eq(filter.getRefractoryPeriod().count(), 20));
	};

	"mask_filter"_test = [] {
		cv::Size resolution(100, 100);
		auto events = dv::data::generate::eventTestSet(1000, resolution);

		cv::Mat mask(resolution, CV_8UC1, cv::Scalar(0));
		cv::Rect roi(0, 0, resolution.width / 2, resolution.height / 2);
		cv::rectangle(mask, roi, cv::Scalar(255), cv::FILLED);

		dv::EventMaskFilter<> filter(mask);
		filter.accept(events);
		dv::EventStore filtered = filter.generateEvents();

		// The filtered data should be within top left quarter
		cv::Rect filteredRoi = dv::boundingRect(filtered);
		expect(le(filteredRoi.br().x, roi.br().x));
		expect(le(filteredRoi.br().y, roi.br().y));

		// Functional type call should result in exactly the same results
		dv::EventStore filteredFunctional;
		dv::maskFilter(events, filteredFunctional, mask);
		cv::Rect filteredFunctionalRoi = dv::boundingRect(filteredFunctional);
		expect(eq(filteredRoi, filteredFunctionalRoi));

		// Wrong mask type should throw an expception
		expect(throws([resolution] {
			cv::Mat mask(resolution, CV_32FC1, cv::Scalar(0));
			dv::EventMaskFilter<> filter(mask);
		}));
	};
}

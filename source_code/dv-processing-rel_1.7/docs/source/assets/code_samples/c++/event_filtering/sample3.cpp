#include <dv-processing/core/filters.hpp>
#include <dv-processing/data/generate.hpp>

#include <opencv2/highgui.hpp>

int main() {
	using namespace std::chrono_literals;

	const cv::Size resolution(200, 200);

	// Initializing 10000 events that are uniformly spaced in pixel area and time
	dv::EventStore events = dv::data::generate::uniformEventsWithinTimeRange(0, 10ms, resolution, 10000);

	// Initialize refractory period filter with 1-millisecond period
	dv::RefractoryPeriodFilter filter(resolution, 1ms);

	// Pass events to the filter
	filter.accept(events);

	// Call generate events to apply the filter
	const dv::EventStore filtered = filter.generateEvents();

	// Print out the number of events after filtering
	std::cout << "Filtered [" << filtered.size() << "] events out of [" << events.size() << "]" << std::endl;

	return 0;
}

#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>

int main() {
	using namespace std::chrono_literals;

	// Generate 10 events with time range [10000; 20000]
	const auto store = dv::data::generate::uniformEventsWithinTimeRange(10000, 10ms, cv::Size(100, 100), 10);

	// Get all events with timestamp above 12500, it will be 13000 and up
	dv::EventStore eventsAfterTimestamp = store.sliceTime(12'500);

	// Print the timestamp ranges
	std::cout << "1. " << eventsAfterTimestamp << std::endl;

	// Slice event within time range [12000; 16000); the end time is exclusive
	const dv::EventStore eventsInRange = store.sliceTime(12'000, 16'000);

	// Print the timestamp ranges; It will print that range is [12000; 15000] since end time is exclusive and
	// event at timestamp 16000 is not going to be included.
	std::cout << "2. " << eventsInRange << std::endl;

	return 0;
}

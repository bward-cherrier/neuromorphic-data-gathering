#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>

int main() {
	using namespace std::chrono_literals;

	// Add 10 event with timestamps in range [10000; 20000]
	const auto store = dv::data::generate::uniformEventsWithinTimeRange(10000, 10ms, cv::Size(100, 100), 10);

	// Get all events beyond and including index 5
	dv::EventStore eventsAfterIndex = store.slice(5);
	std::cout << "1. " << eventsAfterIndex << std::endl;

	// Get 3 events starting with index 2
	dv::EventStore eventsInRange = store.slice(2, 3);
	std::cout << "2. " << eventsInRange << std::endl;

	// Use sliceBack to retrieve event from the end; this call will retrieve last 3 events
	dv::EventStore lastEvents = store.sliceBack(3);
	std::cout << "3. " << lastEvents << std::endl;

	return 0;
}

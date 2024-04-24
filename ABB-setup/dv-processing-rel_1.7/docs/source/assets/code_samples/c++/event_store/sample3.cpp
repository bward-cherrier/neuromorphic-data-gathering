#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>

int main() {
	using namespace std::chrono_literals;

	// Generate 10 events with timestamps in range [10000; 20000]
	const auto store1 = dv::data::generate::uniformEventsWithinTimeRange(10000, 10ms, cv::Size(100, 100), 10);

	// Generate second event store with 10 events with timestamps in range [20000; 29000] to the second store
	const auto store2 = dv::data::generate::uniformEventsWithinTimeRange(20000, 10ms, cv::Size(100, 100), 10);

	// Final event store which will contain all events
	dv::EventStore finalStore;

	// Add the events into the final store; this operation is shallow, so no data copies
	// are performed, but the underlying data has shared ownership between all stores
	finalStore.add(store1);
	finalStore.add(store2);

	// Print specific information on what we contain in the event store
	std::cout << finalStore << std::endl;

	return 0;
}

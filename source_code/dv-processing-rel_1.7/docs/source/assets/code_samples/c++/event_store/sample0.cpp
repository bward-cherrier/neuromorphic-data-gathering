#include <dv-processing/core/core.hpp>
#include <dv-processing/core/utils.hpp>

int main() {
	// Initialize an empty store
	dv::EventStore store;

	// Get the current timestamp
	const int64_t timestamp = dv::now();

	// Add some events into the event store
	// This allocates and inserts events at the back, the function arguments are:
	// timestamp, x, y, polarity
	store.emplace_back(timestamp, 0, 0, true);
	store.emplace_back(timestamp + 1000, 1, 1, false);
	store.emplace_back(timestamp + 2000, 2, 2, false);
	store.emplace_back(timestamp + 3000, 3, 3, true);

	// Perform time-based slicing of event store, the output event store "sliced" will contain
	// the second and third events from above. The end timestamp (second argument) is 2001, since start
	// timestamp (first argument) is inclusive and timestamp is exclusive, so 1 is added.
	const dv::EventStore sliced = store.sliceTime(timestamp + 1000, timestamp + 2001);

	// This should print two events
	for (const dv::Event &ev : sliced) {
		std::cout << fmt::format("Sliced event [{}, {}, {}, {}]", ev.timestamp(), ev.x(), ev.y(), ev.polarity())
				  << std::endl;
	}

	return 0;
}

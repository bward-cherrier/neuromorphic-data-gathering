#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>

int main() {
	// Use this namespace to enable literal time expression from the chrono library
	using namespace std::chrono_literals;

	// Initialize slicer, it will have no jobs at this time
	dv::EventStreamSlicer slicer;

	// Register this method to be called every 33 millisecond worth of event data
	slicer.doEveryTimeInterval(33ms, [](const dv::EventStore &events) {
		std::cout << "* Received events time-based slicing: " << events << std::endl;
	});

	// Register this method to be called every 100 events
	slicer.doEveryNumberOfElements(100, [](const dv::EventStore &events) {
		std::cout << "# Received events in number-based slicing: " << events << std::endl;
	});

	// Generate 1000 events within 2 second interval. These will be sliced correctly by the slicer.
	const dv::EventStore store = dv::data::generate::uniformEventsWithinTimeRange(0, 2s, cv::Size(100, 100), 1000);

	// Now push the store into the slicer, the data contents within the store
	// can be arbitrary, the slicer implementation takes care of correct slicing
	// algorithm and calls the previously registered callbacks accordingly.
	slicer.accept(store);

	return 0;
}

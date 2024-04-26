#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>

int main() {
	// Use this namespace to enable literal time expression from the chrono library
	using namespace std::chrono_literals;

	// Initialize slicer, it will have no jobs at this time
	dv::EventStreamSlicer slicer;

	// Register this method to be called every 33 millisecond worth of event data
	int timeJobId = slicer.doEveryTimeInterval(33ms, [](const dv::EventStore &events) {
		std::cout << "* Received events time-based slicing: " << events << std::endl;
	});

	// Register this method to be called every 100 events
	int numberJobId = slicer.doEveryNumberOfElements(100, [](const dv::EventStore &events) {
		std::cout << "# Received events in number-based slicing: " << events << std::endl;
	});

	// Implement data generation; The following loop will generate 10 packets of events,
	// each containing 100 events within 20 millisecond duration.
	for (int i = 0; i < 10; i++) {
		// Generate 100 events within 20 millisecond interval. These will be sliced correctly by the slicer.
		const auto store = dv::data::generate::uniformEventsWithinTimeRange(i * 20'000, 20ms, cv::Size(100, 100), 100);

		// Now push the store into the slicer, the data contents within the store
		// can be arbitrary, the slicer implementation takes care of correct slicing
		// algorithm and calls the previously registered callbacks accordingly.
		slicer.accept(store);

		// When a packet with index 5 is reached, modify the parameters
		if (i == 5) {
			// Modify time range to 10 milliseconds instead of 33
			slicer.modifyTimeInterval(timeJobId, 10ms);
			// Modify number to 200 instead of 100
			slicer.modifyNumberInterval(numberJobId, 200);
		}
	}
}

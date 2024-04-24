#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/core/event.hpp"
#include "../../include/dv-processing/data/utilities.hpp"

#include "boost/ut.hpp"

static dv::EventStore generateEvents(size_t eventAmount, int64_t timestamp, int64_t timestampIncrement) {
	// Event Store of event over the last one second
	dv::EventStore store;

	// Setup sample data
	for (size_t i = 0; i < eventAmount; i++) {
		store.emplace_back(timestamp, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		timestamp += timestampIncrement;
	}
	return store;
}

template<class Store>
void sliceBackTest(Store &store) {
	using namespace boost::ut;
	Store slice1       = store.sliceBack(100);
	size_t eventAmount = store.size();

	expect(eq(slice1.size(), 100UL));
	expect(eq(slice1.getHighestTime(), store.getHighestTime()));
	expect(neq(slice1.getLowestTime(), store.getLowestTime()));

	Store slice2 = store.sliceBack((static_cast<size_t>(2 * eventAmount)));

	expect(eq(slice2.size(), store.size()));
	expect(eq(slice2.getHighestTime(), store.getHighestTime()));
	expect(eq(slice2.getLowestTime(), store.getLowestTime()));

	// The slice should be empty at an invalid region
	Store emptySlice = store.sliceBack(0);
	expect(emptySlice.isEmpty());
	expect(eq(emptySlice.getLowestTime(), 0));
	expect(eq(emptySlice.getHighestTime(), 0));
}

template<class Store>
void sliceBySizeTest(Store &store, int64_t timestampIncrement) {
	using namespace boost::ut;
	const int64_t startTime     = store.getLowestTime();
	const int64_t endTime       = store.getHighestTime();
	const size_t numberOfEvents = store.size();

	// Sanity checking
	expect(eq(startTime, store.front().timestamp()));
	expect(eq(endTime, store.back().timestamp()));

	const Store frontSlice = store.slice(0, 10);
	expect(eq(frontSlice.getLowestTime(), startTime));
	expect(eq(frontSlice.getHighestTime(), startTime + (9 * timestampIncrement)));

	expect(throws([&store, numberOfEvents] {
		Store overflowSlice = store.slice(0, numberOfEvents * 2);
	})) << "Throws overflow exception";

	const Store fullSlice = store.slice(static_cast<size_t>(0));
	expect(eq(fullSlice.size(), store.size()));

	const Store backSlice = store.sliceBack(100);
	expect(eq(backSlice.size(), 100UL));
	expect(eq(backSlice.getHighestTime(), store.getHighestTime()));
	expect(eq(backSlice.getLowestTime(), endTime - (99 * timestampIncrement)));
}

int main() {
	using namespace boost::ut;

	const size_t numberOfEvents      = 100000;
	const int64_t timestampIncrement = 10000LL;
	const int64_t timestamp          = 1000LL;

	"sharding"_test = [] {
		dv::EventStore store;
		store.setShardCapacity(0);
		expect(eq(store.getShardCapacity(), 1));
		store.setShardCapacity(3);
		expect(eq(store.getShardCapacity(), 3));
		store.emplace_back(1000, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);

		// It should contain two data shards
		expect(eq(store.getShardCount(), 2ULL));
		expect(eq(store.size(), 5ULL));
		expect(eq(store.getLowestTime(), 1000LL));
		expect(eq(store.getHighestTime(), 1100LL));

		dv::EventStore store2;
		store2.setShardCapacity(2);
		store2.emplace_back(1200, 0, 0, true);
		store2.emplace_back(1300, 0, 0, true);
		expect(eq(store2.getShardCount(), 1ULL));
		expect(eq(store2.size(), 2ULL));
		expect(eq(store2.getLowestTime(), 1200LL));
		expect(eq(store2.getHighestTime(), 1300LL));

		store.add(store2);
		expect(eq(store.size(), 7ULL));
		expect(eq(store.getShardCount(), 3ULL));
		expect(eq(store.getLowestTime(), 1000LL));
		expect(eq(store.getHighestTime(), 1300LL));

		dv::EventStore largeShardStore;
		largeShardStore.setShardCapacity(10000);

		// This will allocate a new shard with given capacity
		largeShardStore.emplace_back(1000, 0, 0, true);
		largeShardStore.add(store);
		expect(eq(largeShardStore.size(), 8ULL));
		// Now this should merge the data into a single shard to reduce fragmentation
		expect(eq(largeShardStore.getShardCount(), 1ULL));
		expect(eq(largeShardStore.getLowestTime(), 1000LL));
		expect(eq(largeShardStore.getHighestTime(), 1300LL));
	};

	"shard_slicing"_test = [] {
		dv::EventStore store;
		store.emplace_back(1000, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);
		store.emplace_back(1200, 0, 0, true);
		store.emplace_back(1300, 0, 0, true);
		store.emplace_back(1400, 0, 0, true);

		auto slice = store.slice(1, 2);

		dv::EventStore store2;
		store2.emplace_back(1500, 0, 0, true);
		store2.emplace_back(1600, 0, 0, true);

		slice.add(store2);
		expect(eq(slice.size(), 4));
		expect(eq(slice.at(0).timestamp(), 1100));
		expect(eq(slice.at(1).timestamp(), 1200));
		expect(eq(slice.at(2).timestamp(), 1500));
		expect(eq(slice.at(3).timestamp(), 1600));
		expect(eq(slice.getLowestTime(), 1100));
		expect(eq(slice.getHighestTime(), 1600));
	};

	"add_events"_test = [] {
		dv::EventStore store;
		store.emplace_back(1000, 0, 0, true);
		store.emplace_back(1100, 0, 0, true);

		expect(eq(store.size(), 2ULL));
		expect(eq(store.getLowestTime(), 1000LL));
		expect(eq(store.getHighestTime(), 1100LL));

		dv::EventStore store2;
		store2.emplace_back(1200, 0, 0, true);
		store2.emplace_back(1300, 0, 0, true);
		expect(eq(store2.size(), 2ULL));
		expect(eq(store2.getLowestTime(), 1200LL));
		expect(eq(store2.getHighestTime(), 1300LL));

		store.add(store2);
		expect(eq(store.size(), 4ULL));
		expect(eq(store.getLowestTime(), 1000LL));
		expect(eq(store.getHighestTime(), 1300LL));
	};

	"data_partial_merging"_test = [] {
		dv::PartialEventData<dv::Event, dv::EventPacket> shard;
		dv::PartialEventData<dv::Event, dv::EventPacket> anotherShard;

		// Empty shard merging with itself. Technically that's allowed.
		expect(shard.merge(shard));

		shard._unsafe_addEvent(dv::Event(1000, 0, 0, true));
		// Merging non-empty with an empty shard, should be fine.
		expect(shard.merge(anotherShard));

		// Construct the specific case of const referenced data partial and test it's specific properties
		dv::EventPacket packet;
		packet.elements.emplace_back(1000, 0, 0, true);
		dv::PartialEventData<dv::Event, dv::EventPacket> constShard(std::make_shared<const dv::EventPacket>(packet));

		// Const referenced data shard should not have any capacity and should not be able to merge with other shards
		expect(eq(constShard.availableCapacity(), 0ULL));
		expect(!constShard.canStoreMoreEvents());
		expect(!constShard.merge(shard));
	};

	"slice_back"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);
		sliceBackTest(store);
	};

	"slicing_by_size"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);
		expect(eq(store.front().timestamp(), timestamp));
		sliceBySizeTest(store, timestampIncrement);
	};

	"slicing_by_time"_test = [=] {
		auto store        = generateEvents(numberOfEvents, timestamp, timestampIncrement);
		int64_t startTime = store.getLowestTime();
		int64_t endTime   = store.getHighestTime();

		// End time should slice the last event
		dv::EventStore lastEventSlice = store.sliceTime(endTime);
		expect(eq(lastEventSlice.size(), 1UL));
		expect(eq(lastEventSlice.getLowestTime(), store.getHighestTime()));
		expect(eq(lastEventSlice.getHighestTime(), lastEventSlice.getLowestTime()));

		dv::EventStore emptySlice = store.sliceTime(endTime, startTime);
		expect(emptySlice.isEmpty());

		size_t begin, end;
		dv::EventStore fullSlice = store.sliceTime(startTime, endTime + 1, begin, end);
		expect(eq(fullSlice.getLowestTime(), store.getLowestTime()));
		expect(eq(fullSlice.getHighestTime(), store.getHighestTime()));
		expect(eq(begin, 0UL));
		expect(eq(end, numberOfEvents));

		dv::EventStore nullSlice = store.sliceTime(endTime + 1000, endTime + 10000, begin, end);
		expect(eq(nullSlice.getLowestTime(), 0LL));
		expect(eq(nullSlice.getHighestTime(), 0LL));
		expect(eq(begin, 0UL));
		expect(eq(end, 0UL));
	};

	"store_operators"_test = [=] {
		auto store      = generateEvents(numberOfEvents, timestamp, timestampIncrement);
		int64_t endTime = store.getHighestTime();

		// Should create a copy container of the same events
		dv::EventStore copy;
		copy.add(store);

		expect(eq(copy.size(), numberOfEvents));
		expect(eq(copy.getLowestTime(), store.getLowestTime()));
		expect(eq(copy.getHighestTime(), store.getHighestTime()));

		int64_t myTime = store.getHighestTime() + timestampIncrement;

		// Direct add method call
		copy.emplace_back(myTime, static_cast<int16_t>(0), static_cast<int16_t>(0), true);
		expect(eq(copy.size(), numberOfEvents + 1));
		expect(eq(copy.getHighestTime(), myTime));

		// Stream operator
		myTime += timestampIncrement;
		copy << dv::Event(myTime, 0, 0, true);
		expect(eq(copy.size(), numberOfEvents + 2));
		expect(eq(copy.getHighestTime(), myTime));

		// Adding without incrementing the time should throw an exception
		expect(throws([&] {
			copy.emplace_back(myTime - 1, static_cast<int16_t>(0), static_cast<int16_t>(0), true);
		})) << "Throws out of order exception";

		// Highest time should remain the same
		expect(eq(copy.getHighestTime(), myTime));
		expect(eq(copy.size(), numberOfEvents + 2));

		dv::EventStore slice = copy.sliceTime(myTime);
		expect(eq(slice.size(), 1UL));

		dv::EventStore duplicate = store + copy.sliceTime(endTime + 1);
		expect(eq(duplicate.size(), numberOfEvents + 2));

		expect(throws([&] {
			store += store;
		})) << "Throws out of order exception";
	};

	"coord_hash"_test = [] {
		uint32_t hash0 = dv::coordinateHash(0, 0);
		uint32_t hash1 = dv::coordinateHash(10, 0);
		uint32_t hash2 = dv::coordinateHash(0, 10);

		expect(eq(hash0, hash0));
		expect(eq(hash1, hash1));
		expect(eq(hash2, hash2));
		expect(neq(hash2, hash0));
		expect(neq(hash2, hash1));

		uint32_t hash4 = dv::coordinateHash(1, 1);
		uint32_t hash5 = dv::coordinateHash(-1, -1);
		expect(neq(hash4, hash5));
	};

	"filtering_and_scaling"_test = [] {
		dv::EventStore events;

		cv::Rect empty = dv::boundingRect(events);
		expect(eq(empty.area(), 0));

		events.emplace_back(0, static_cast<int16_t>(0), static_cast<int16_t>(0), false);
		events.emplace_back(1, static_cast<int16_t>(10), static_cast<int16_t>(0), false);
		events.emplace_back(2, static_cast<int16_t>(12), static_cast<int16_t>(12), false);
		events.emplace_back(3, static_cast<int16_t>(5), static_cast<int16_t>(0), true);
		events.emplace_back(4, static_cast<int16_t>(18), static_cast<int16_t>(21), true);

		cv::Rect roi(10, 10, 20, 20);
		dv::EventStore filtered;
		dv::roiFilter(events, filtered, roi);

		expect(eq(filtered.size(), 2UL));

		cv::Rect bounds = dv::boundingRect(events);
		expect(eq(bounds.x, 0));
		expect(eq(bounds.y, 0));
		expect(eq(bounds.width, 19));
		expect(eq(bounds.height, 22));

		dv::EventStore positives, negatives;
		dv::polarityFilter(events, positives, true);
		expect(eq(positives.size(), 2UL));

		dv::polarityFilter(events, negatives, false);
		expect(eq(negatives.size(), 3UL));

		double scale = 2.0;
		dv::EventStore scaled;
		dv::scale(events, scaled, scale, scale);
		cv::Rect scaledBounds = dv::boundingRect(scaled);
		expect(eq(scaledBounds.width, 10));
		expect(eq(scaledBounds.height, 11));
	};

	"erase_front"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(0, 10);
		expect(eq(eventStore.size(), priorSize - 10));
	};

	"erase_front_middle"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(2000, 10);
		expect(eq(eventStore.size(), priorSize - 10));
	};

	"erase_back"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(990, 10);
		expect(eq(eventStore.size(), priorSize - 10));
	};

	"erase_back_middle"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(2990, 10);
		expect(eq(eventStore.size(), priorSize - 10));
	};

	"erase_within_a_packet"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(2350, 10);
		expect(eq(eventStore.size(), priorSize - 10));
	};

	"erase_within_two_packets"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(0, 1500);
		expect(eq(eventStore.size(), priorSize - 1500));
	};

	"erase_within_two_packets"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(0, 5500);
		expect(eq(eventStore.size(), priorSize - 5500));
	};

	"erase_within_large_range"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(1500, 8000);
		expect(eq(eventStore.size(), priorSize - 8000));
	};

	"erase_nothing"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(1000, 0);
		expect(eq(eventStore.size(), priorSize));
	};

	"erase_everything"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();
		eventStore.erase(0, priorSize);
		expect(eq(eventStore.size(), static_cast<size_t>(0)));
	};

	"erase_out_of_range"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		size_t priorSize          = eventStore.size();

		expect(throws([&eventStore, priorSize] {
			eventStore.erase(1500, priorSize);
		}));

		expect(throws([&eventStore, priorSize] {
			eventStore.erase(priorSize + 1500, priorSize);
		}));

		expect(throws([&eventStore, priorSize] {
			eventStore.erase(priorSize + 1500, 0);
		}));
	};

	"erase_time_last"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		eventStore.eraseTime(eventStore.getLowestTime(), eventStore.getHighestTime());
		expect(eq(eventStore.size(), static_cast<size_t>(1)));
	};

	"erase_time_everything"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		eventStore.eraseTime(eventStore.getLowestTime(), eventStore.getHighestTime() + 1);
		expect(eq(eventStore.size(), static_cast<size_t>(0)));
	};

	"erase_time_first_and_last"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		eventStore.eraseTime(eventStore.getLowestTime() + 1, eventStore.getHighestTime());
		expect(eq(eventStore.size(), static_cast<size_t>(2)));
		expect(eq(eventStore.getLowestTime(), store.getLowestTime()));
		expect(eq(eventStore.getHighestTime(), store.getHighestTime()));
		expect(eq(eventStore.at(0).timestamp(), store.getLowestTime()));
		expect(eq(eventStore.at(1).timestamp(), store.getHighestTime()));
	};

	"erase_time_half"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		eventStore.eraseTime(
			eventStore.getLowestTime(), (eventStore.getLowestTime() + eventStore.getHighestTime()) / 2);
		expect(eq(eventStore.size(), store.size() / 2));
		expect(eq(eventStore.at(0).timestamp(), store.at(store.size() / 2).timestamp()));
	};

	"event_at_out_of_bounds"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		expect(throws([&] {
			dv::Event ev = store.at(numberOfEvents);
			// Suppress unused warning
			(void) ev;
		}));
	};

	"retain_duration"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		dv::Duration duration     = store.duration();
		eventStore.retainDuration(duration / 2);
		expect(lt(eventStore.duration().count(), store.duration().count()));
		expect(ge(eventStore.duration().count(), (duration / 2).count()));
	};

	"retain_duration_full"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		dv::EventStore eventStore = store.slice(static_cast<size_t>(0));
		dv::Duration duration     = store.duration();
		eventStore.retainDuration(duration);
		expect(eq(eventStore.duration().count(), store.duration().count()));
		// Even with zero duration, it should maintain some events
		eventStore.retainDuration(dv::Duration(0));
		expect(!eventStore.isEmpty());
		expect(le(eventStore.size(), store.size()));
	};

	"slice_rate"_test = [] {
		const size_t eventAmount         = 1'000'000;
		const int64_t timestampIncrement = 1;
		const int64_t timestamp          = 1'000;
		auto store                       = generateEvents(eventAmount, timestamp, timestampIncrement);

		// Initial sanity checks
		expect(eq(store.size(), 1'000'000ULL));
		expect(eq(store.duration().count(), 999'999LL));
		// The rate is 1000000/999999 = 1000001
		expect(eq(store.rate(), 1000001._d));

		// Reduce amount of event by half
		const dv::EventStore rateLimited = store.sliceRate(500000.0);
		expect(lt(rateLimited.duration().count(), 500'000LL));
		expect(gt(rateLimited.duration().count(), 499'990LL));
		expect(eq(rateLimited.size(), 499'999ULL));

		// Invalid rates for slicing
		expect(throws([&store] {
			store.sliceRate(0.0);
		}));
		expect(throws([&store] {
			store.sliceRate(-100.0);
		}));

		// Slicing above rate should return the same events
		expect(eq(store.sliceRate(2e+6).size(), store.size()));

		expect(nothrow([] {
			expect(eq(dv::EventStore().sliceRate(1e+6).size(), 0ULL));
		}));
	};

	"timestamp_check"_test = [=] {
		auto store = generateEvents(numberOfEvents, timestamp, timestampIncrement);

		expect(store.isWithinStoreTimeRange(store.getLowestTime()));
		expect(store.isWithinStoreTimeRange(store.getHighestTime()));
		expect(store.isWithinStoreTimeRange((store.getHighestTime() + store.getLowestTime()) / 2));
		expect(!store.isWithinStoreTimeRange(store.getHighestTime() + 100));
		expect(!store.isWithinStoreTimeRange(store.getLowestTime() - 100));
	};

	"emplace_back"_test = [=] {
		// Event Store of event over the last one second
		dv::EventStore store;

		int16_t position = 0;
		dv::Event event(1000LL, position, position, false);
		store.push_back(event);
		expect(eq(store.size(), 1ULL));

		store.emplace_back(1100LL, position, position, false);
		expect(eq(store.size(), 2ULL));
		expect(eq(store.getLowestTime(), 1000LL));
		expect(eq(store.getHighestTime(), 1100LL));

		dv::AddressableEventStorage<dv::DepthEvent, dv::DepthEventPacket> depthEventStore;
		uint16_t depthValue = 1000;
		depthEventStore.emplace_back(1000LL, position, position, false, depthValue);
		depthEventStore.emplace_back(1100LL, position, position, false, depthValue);
		expect(eq(depthEventStore.size(), 2ULL));
		expect(eq(depthEventStore.getLowestTime(), 1000LL));
		expect(eq(depthEventStore.getHighestTime(), 1100LL));
	};

	"event_rate"_test = [] {
		dv::EventStore store;

		expect(eq(store.rate(), 0.0_d));

		store.emplace_back(100'000, 0, 0, true);
		store.emplace_back(120'000, 0, 0, true);
		store.emplace_back(140'000, 0, 0, true);
		store.emplace_back(160'000, 0, 0, true);
		store.emplace_back(180'000, 0, 0, true);
		store.emplace_back(200'000, 0, 0, true);

		expect(eq(store.rate(), 60.0_d));
	};

	"event_packet_implicit_conversion"_test = [] {
		dv::EventPacket packet;
		packet.elements.emplace_back(100'000, 0, 0, true);
		packet.elements.emplace_back(120'000, 0, 0, true);
		packet.elements.emplace_back(140'000, 0, 0, true);
		packet.elements.emplace_back(160'000, 0, 0, true);
		packet.elements.emplace_back(180'000, 0, 0, true);
		packet.elements.emplace_back(200'000, 0, 0, true);

		std::shared_ptr<const dv::EventPacket> packetPtr = std::make_shared<const dv::EventPacket>(packet);

		dv::EventStore store;
		store = packetPtr;
		expect(eq(store.getLowestTime(), 100'000));
		expect(eq(store.getHighestTime(), 200'000));
		expect(eq(store.size(), 6));

		store = packetPtr;
		expect(eq(store.getLowestTime(), 100'000));
		expect(eq(store.getHighestTime(), 200'000));
		expect(eq(store.size(), 6));

		dv::EventPacket packet2;
		packet2.elements.emplace_back(210'000, 0, 0, true);

		store.add(dv::EventStore(std::make_shared<const dv::EventPacket>(packet2)));
		expect(eq(store.getLowestTime(), 100'000));
		expect(eq(store.getHighestTime(), 210'000));
		expect(eq(store.size(), 7));
	};

	"convert_to_eigen"_test = [] {
		dv::EventStore store;
		store.setShardCapacity(2);
		store.emplace_back(100'000, 0, 1, true);
		store.emplace_back(120'000, 2, 3, false);
		store.emplace_back(140'000, 4, 5, true);
		store.emplace_back(160'000, 6, 7, false);
		store.emplace_back(180'000, 8, 9, true);
		store.emplace_back(200'000, 10, 11, false);

		const auto timestamps = store.timestamps();
		expect(eq(timestamps.rows(), 6));
		expect(eq(timestamps.cols(), 1));
		expect(eq(timestamps[0], 100'000));
		expect(eq(timestamps[1], 120'000));
		expect(eq(timestamps[2], 140'000));
		expect(eq(timestamps[3], 160'000));
		expect(eq(timestamps[4], 180'000));
		expect(eq(timestamps[5], 200'000));

		const auto coordinates = store.coordinates();
		expect(eq(coordinates.rows(), 6));
		expect(eq(coordinates.cols(), 2));
		expect(eq(coordinates(0, 0), 0));
		expect(eq(coordinates(1, 0), 2));
		expect(eq(coordinates(2, 0), 4));
		expect(eq(coordinates(3, 0), 6));
		expect(eq(coordinates(4, 0), 8));
		expect(eq(coordinates(5, 0), 10));
		expect(eq(coordinates(0, 1), 1));
		expect(eq(coordinates(1, 1), 3));
		expect(eq(coordinates(2, 1), 5));
		expect(eq(coordinates(3, 1), 7));
		expect(eq(coordinates(4, 1), 9));
		expect(eq(coordinates(5, 1), 11));

		const auto polarities = store.polarities();
		expect(eq(polarities.rows(), 6));
		expect(eq(polarities.cols(), 1));
		expect(eq(polarities[0], 1));
		expect(eq(polarities[1], 0));
		expect(eq(polarities[2], 1));
		expect(eq(polarities[3], 0));
		expect(eq(polarities[4], 1));
		expect(eq(polarities[5], 0));

		const auto eigenEvents = store.eigen();
		expect(eq(eigenEvents.timestamps.rows(), 6));
		expect(eq(eigenEvents.timestamps.cols(), 1));
		expect(eq(eigenEvents.timestamps[0], 100'000));
		expect(eq(eigenEvents.timestamps[1], 120'000));
		expect(eq(eigenEvents.timestamps[2], 140'000));
		expect(eq(eigenEvents.timestamps[3], 160'000));
		expect(eq(eigenEvents.timestamps[4], 180'000));
		expect(eq(eigenEvents.timestamps[5], 200'000));
		expect(eq(eigenEvents.coordinates.rows(), 6));
		expect(eq(eigenEvents.coordinates.cols(), 2));
		expect(eq(eigenEvents.coordinates(0, 0), 0));
		expect(eq(eigenEvents.coordinates(1, 0), 2));
		expect(eq(eigenEvents.coordinates(2, 0), 4));
		expect(eq(eigenEvents.coordinates(3, 0), 6));
		expect(eq(eigenEvents.coordinates(4, 0), 8));
		expect(eq(eigenEvents.coordinates(5, 0), 10));
		expect(eq(eigenEvents.coordinates(0, 1), 1));
		expect(eq(eigenEvents.coordinates(1, 1), 3));
		expect(eq(eigenEvents.coordinates(2, 1), 5));
		expect(eq(eigenEvents.coordinates(3, 1), 7));
		expect(eq(eigenEvents.coordinates(4, 1), 9));
		expect(eq(eigenEvents.coordinates(5, 1), 11));
		expect(eq(eigenEvents.polarities.rows(), 6));
		expect(eq(eigenEvents.polarities.cols(), 1));
		expect(eq(eigenEvents.polarities[0], 1));
		expect(eq(eigenEvents.polarities[1], 0));
		expect(eq(eigenEvents.polarities[2], 1));
		expect(eq(eigenEvents.polarities[3], 0));
		expect(eq(eigenEvents.polarities[4], 1));
		expect(eq(eigenEvents.polarities[5], 0));
	};

	"empty_store_eigen_conversion"_test = [] {
		dv::EventStore empty;
		const auto timestamps = empty.timestamps();
		expect(eq(timestamps.size(), 0));
		const auto coordinates = empty.coordinates();
		expect(eq(coordinates.size(), 0));
		const auto polarities = empty.polarities();
		expect(eq(polarities.size(), 0));
		const auto eigenEvents = empty.eigen();
		expect(eq(eigenEvents.timestamps.size(), 0));
		expect(eq(eigenEvents.coordinates.size(), 0));
		expect(eq(eigenEvents.polarities.size(), 0));
	};

	"packet_conversion"_test = [] {
		dv::EventStore store;
		expect(store.isEmpty());

		const dv::EventPacket empty = store.toPacket();
		expect(eq(empty.elements.size(), 0));

		store.setShardCapacity(2);
		store.emplace_back(100'000, 0, 1, true);
		store.emplace_back(120'000, 2, 3, false);
		store.emplace_back(140'000, 4, 5, true);
		store.emplace_back(160'000, 6, 7, false);
		store.emplace_back(180'000, 8, 9, true);
		store.emplace_back(200'000, 10, 11, false);

		expect(eq(store.getShardCount(), 3));

		const dv::EventPacket packet = store.toPacket();
		expect(eq(packet.elements.size(), 6));

		expect(eq(packet.elements[0].timestamp(), 100'000));
		expect(eq(packet.elements[1].timestamp(), 120'000));
		expect(eq(packet.elements[2].timestamp(), 140'000));
		expect(eq(packet.elements[3].timestamp(), 160'000));
		expect(eq(packet.elements[4].timestamp(), 180'000));
		expect(eq(packet.elements[5].timestamp(), 200'000));

		expect(eq(packet.elements[0].x(), 0));
		expect(eq(packet.elements[1].x(), 2));
		expect(eq(packet.elements[2].x(), 4));
		expect(eq(packet.elements[3].x(), 6));
		expect(eq(packet.elements[4].x(), 8));
		expect(eq(packet.elements[5].x(), 10));

		expect(eq(packet.elements[0].y(), 1));
		expect(eq(packet.elements[1].y(), 3));
		expect(eq(packet.elements[2].y(), 5));
		expect(eq(packet.elements[3].y(), 7));
		expect(eq(packet.elements[4].y(), 9));
		expect(eq(packet.elements[5].y(), 11));

		expect(eq(packet.elements[0].polarity(), true));
		expect(eq(packet.elements[1].polarity(), false));
		expect(eq(packet.elements[2].polarity(), true));
		expect(eq(packet.elements[3].polarity(), false));
		expect(eq(packet.elements[4].polarity(), true));
		expect(eq(packet.elements[5].polarity(), false));
	};

	"depth_event_packet_conversion"_test = [] {
		dv::DepthEventStore store;
		expect(store.isEmpty());

		const dv::DepthEventPacket empty = store.toPacket();
		expect(eq(empty.elements.size(), 0));

		store.setShardCapacity(2);
		store.emplace_back(100'000, 0, 1, true, 100);
		store.emplace_back(120'000, 2, 3, false, 200);
		store.emplace_back(140'000, 4, 5, true, 300);
		store.emplace_back(160'000, 6, 7, false, 400);
		store.emplace_back(180'000, 8, 9, true, 500);
		store.emplace_back(200'000, 10, 11, false, 600);

		expect(eq(store.getShardCount(), 3));

		const dv::DepthEventPacket packet = store.toPacket();
		expect(eq(packet.elements.size(), 6));

		expect(eq(packet.elements[0].timestamp(), 100'000));
		expect(eq(packet.elements[1].timestamp(), 120'000));
		expect(eq(packet.elements[2].timestamp(), 140'000));
		expect(eq(packet.elements[3].timestamp(), 160'000));
		expect(eq(packet.elements[4].timestamp(), 180'000));
		expect(eq(packet.elements[5].timestamp(), 200'000));

		expect(eq(packet.elements[0].x(), 0));
		expect(eq(packet.elements[1].x(), 2));
		expect(eq(packet.elements[2].x(), 4));
		expect(eq(packet.elements[3].x(), 6));
		expect(eq(packet.elements[4].x(), 8));
		expect(eq(packet.elements[5].x(), 10));

		expect(eq(packet.elements[0].y(), 1));
		expect(eq(packet.elements[1].y(), 3));
		expect(eq(packet.elements[2].y(), 5));
		expect(eq(packet.elements[3].y(), 7));
		expect(eq(packet.elements[4].y(), 9));
		expect(eq(packet.elements[5].y(), 11));

		expect(eq(packet.elements[0].polarity(), true));
		expect(eq(packet.elements[1].polarity(), false));
		expect(eq(packet.elements[2].polarity(), true));
		expect(eq(packet.elements[3].polarity(), false));
		expect(eq(packet.elements[4].polarity(), true));
		expect(eq(packet.elements[5].polarity(), false));

		expect(eq(packet.elements[0].depth(), 100));
		expect(eq(packet.elements[1].depth(), 200));
		expect(eq(packet.elements[2].depth(), 300));
		expect(eq(packet.elements[3].depth(), 400));
		expect(eq(packet.elements[4].depth(), 500));
		expect(eq(packet.elements[5].depth(), 600));
	};

	"stream_to_string"_test = [] {
		dv::EventStore store;

		{
			std::stringstream ss;
			ss << store;
			expect(eq(
				ss.str().compare("EventStore containing 0 events within 0µs duration; time range within [0; 0]"), 0));
		}
		store.emplace_back(1000, 0, 0, false);
		{
			std::stringstream ss;
			ss << store;
			expect(eq(
				ss.str().compare("EventStore containing 1 events within 0µs duration; time range within [1000; 1000]"),
				0));
		}
		store.emplace_back(2000, 0, 0, false);
		{
			std::stringstream ss;
			ss << store;
			expect(eq(ss.str().compare(
						  "EventStore containing 2 events within 1000µs duration; time range within [1000; 2000]"),
				0));
		}
	};

	return EXIT_SUCCESS;
}

import unittest
import dv_processing as dv
import numpy as np


def generate_events(amount, timestamp, time_increment):
    store = dv.EventStore()

    for i in range(amount):
        store.push_back(timestamp, 0, 0, False)
        timestamp = timestamp + time_increment

    return store


class EventStoreTest(unittest.TestCase):

    def test_size(self):
        events = dv.EventStore()
        self.assertEqual(events.size(), 0)

        events.push_back(10, 0, 0, False)
        self.assertEqual(events.size(), 1)

        events.push_back((20, 0, 0, True))
        self.assertEqual(events.size(), 2)

        events.push_back(dv.Event(30, 0, 0, False))
        self.assertEqual(events.size(), 3)

    def test_slice_back(self):
        store = generate_events(10000, 1000, 10000)
        start_time = store.getLowestTime()
        end_time = store.getHighestTime()

        self.assertEqual(store.front().timestamp(), start_time)
        self.assertEqual(store.back().timestamp(), end_time)

        slice1 = store.sliceBack(100)
        self.assertEqual(slice1.getHighestTime(), store.getHighestTime())
        self.assertNotEqual(slice1.getLowestTime(), store.getLowestTime())

        slice2 = store.sliceBack(store.size() * 2)
        self.assertEqual(slice2.getHighestTime(), store.getHighestTime())
        self.assertEqual(slice2.getLowestTime(), store.getLowestTime())
        self.assertEqual(slice2.size(), store.size())

        empty_slice = store.sliceBack(0)
        self.assertTrue(empty_slice.isEmpty())
        self.assertEqual(empty_slice.getHighestTime(), 0)
        self.assertEqual(empty_slice.getLowestTime(), 0)

    def test_slice_by_time(self):
        store = generate_events(10000, 1000, 10000)
        start_time = store.getLowestTime()
        end_time = store.getHighestTime()

        last_event_slice = store.sliceTime(end_time)
        self.assertEqual(last_event_slice.size(), 1)
        self.assertEqual(last_event_slice.getLowestTime(), store.getHighestTime())
        self.assertEqual(last_event_slice.getHighestTime(), last_event_slice.getLowestTime())

        empty_slice = store.sliceTime(end_time, start_time)
        self.assertTrue(empty_slice.isEmpty())

        full_slice = store.sliceTime(start_time, end_time + 1)
        self.assertEqual(full_slice.getLowestTime(), store.getLowestTime())
        self.assertEqual(full_slice.getHighestTime(), store.getHighestTime())
        self.assertEqual(full_slice.size(), store.size())

    def test_store_add(self):
        store = generate_events(10000, 1000, 0)

        copy = dv.EventStore()
        copy.add(store)
        self.assertEqual(copy.size(), store.size())
        self.assertEqual(copy.getLowestTime(), store.getLowestTime())
        self.assertEqual(copy.getHighestTime(), store.getHighestTime())

        copy.add(store)
        self.assertEqual(copy.size(), store.size() * 2)
        self.assertEqual(copy.getLowestTime(), store.getLowestTime())
        self.assertEqual(copy.getHighestTime(), store.getHighestTime())

    def test_coord_hash(self):
        hash0 = dv.coordinateHash(0, 0)
        hash1 = dv.coordinateHash(10, 0)
        hash2 = dv.coordinateHash(0, 100)
        self.assertNotEqual(hash0, hash1)
        self.assertNotEqual(hash0, hash2)
        self.assertNotEqual(hash1, hash2)

        hash4 = dv.coordinateHash(1, 1)
        hash5 = dv.coordinateHash(-1, -1)
        self.assertNotEqual(hash4, hash5)

    def test_roi_filtering_bounding(self):
        events = dv.EventStore()
        events.push_back(0, 0, 0, False)
        events.push_back(1, 10, 0, False)
        events.push_back(2, 12, 12, False)
        events.push_back(3, 5, 0, True)
        events.push_back(4, 18, 21, True)

        filtered = dv.roiFilter(events, (10, 10, 20, 20))
        self.assertEqual(filtered.size(), 2)

        bounds = dv.boundingRect(events)
        self.assertEqual(bounds[0], 0)
        self.assertEqual(bounds[1], 0)
        self.assertEqual(bounds[2], 19)
        self.assertEqual(bounds[3], 22)

        positives = dv.polarityFilter(events, True)
        self.assertEqual(positives.size(), 2)

        negatives = dv.polarityFilter(events, False)
        self.assertEqual(negatives.size(), 3)

    def test_erase(self):
        events = generate_events(10000, 1000, 0)
        prior_size = events.size()

        copy = events.slice(0)
        events.erase(0, 10)
        self.assertEqual(events.size(), prior_size - 10)

        # erasing data in the original does not affect the copy
        self.assertEqual(copy.size(), prior_size)

    def test_retain_duration(self):
        events = generate_events(100000, 1000, 10000)
        prior_size = events.size()
        duration = events.duration()

        events.retainDuration(duration / 2)
        self.assertLess(events.size(), prior_size)
        self.assertGreaterEqual(events.duration(), duration / 2)

    def test_event_iterator(self):
        amount = 10000
        events = generate_events(10000, 1000, 10000)
        self.assertEqual(len(events), amount)

        counter = 0
        for event in events:
            self.assertEqual(event.timestamp(), events.at(counter).timestamp())
            self.assertEqual(event.timestamp(), events[counter].timestamp())
            counter += 1
        self.assertEqual(counter, amount)

    def test_array_access(self):
        amount = 10000
        events = generate_events(amount, 1000, 10000)

        events_array = events.numpy()

        self.assertEqual(len(events_array["timestamp"]), amount)
        self.assertEqual(len(events_array["x"]), amount)
        self.assertEqual(len(events_array["y"]), amount)
        self.assertEqual(len(events_array["polarity"]), amount)
        self.assertTrue(isinstance(events_array, np.ndarray))

        timestamps = events.timestamps()
        self.assertEqual(timestamps.shape[0], amount)
        self.assertEqual(np.sum(timestamps), 499960000000)
        self.assertTrue(isinstance(timestamps, np.ndarray))

        coordinates = events.coordinates()
        self.assertEqual(coordinates.shape[0], amount)
        self.assertEqual(coordinates.shape[1], 2)
        self.assertEqual(np.sum(coordinates), 0)
        self.assertTrue(isinstance(coordinates, np.ndarray))

        polarities = events.polarities()
        self.assertEqual(polarities.shape[0], amount)
        self.assertEqual(np.sum(polarities), 0)
        self.assertTrue(isinstance(polarities, np.ndarray))

    def test_event_packet(self):
        event_packet = dv.EventPacket()
        event_packet.elements.append(dv.Event(100, 1, 1, True))
        event_packet.elements.append(dv.Event(100100, 10, 11, False))
        np_array = event_packet.numpy()
        self.assertTrue(isinstance(np_array, np.ndarray))
        self.assertEqual(np_array["timestamp"][0], 100)
        self.assertEqual(np_array["timestamp"][1], 100100)

        self.assertEqual(np_array["x"][0], 1)
        self.assertEqual(np_array["x"][1], 10)

        self.assertEqual(np_array["y"][0], 1)
        self.assertEqual(np_array["y"][1], 11)

        self.assertEqual(np_array["polarity"][0], 1)
        self.assertEqual(np_array["polarity"][1], 0)


if __name__ == '__main__':
    unittest.main()

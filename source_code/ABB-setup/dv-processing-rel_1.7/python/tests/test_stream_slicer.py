import unittest
import dv_processing as dv
import datetime


def generate_events(amount, timestamp, time_increment):
    store = dv.EventStore()

    for i in range(amount):
        store.push_back(timestamp, 0, 0, False)
        timestamp = timestamp + time_increment

    return store


class EventStreamSlicerTest(unittest.TestCase):
    counter = 0

    def increment_counter(self, store):
        value = self.counter + store.size()
        self.counter = value

    def test_number_slicing(self):
        store = generate_events(10000, 0, 1000)
        slicer = dv.EventStreamSlicer()
        self.counter = 0

        job_id = slicer.doEveryNumberOfEvents(1000, self.increment_counter)
        self.assertTrue(slicer.hasJob(job_id))
        slicer.accept(store)
        self.assertEqual(self.counter, store.size())

    def test_time_slicing(self):
        # We need one extra event further in time to let the slicer know
        # that the interval is over
        store = generate_events(10001, 0, 1000)
        slicer = dv.EventStreamSlicer()
        self.counter = 0

        job_id = slicer.doEveryTimeInterval(datetime.timedelta(milliseconds=10), self.increment_counter)
        self.assertTrue(slicer.hasJob(job_id))
        slicer.accept(store)
        self.assertEqual(self.counter, 10000)

        self.counter = 0
        store = generate_events(10001, store.getHighestTime(), 1000)
        slicer.modifyTimeInterval(job_id, datetime.timedelta(milliseconds=100))
        slicer.accept(store)
        self.assertEqual(10001, self.counter)


if __name__ == '__main__':
    unittest.main()

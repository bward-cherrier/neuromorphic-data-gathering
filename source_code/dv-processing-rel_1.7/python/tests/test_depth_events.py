import dv_processing as dv
import unittest
import numpy as np


class DepthEventTests(unittest.TestCase):

    def test_data_structure(self):
        depth_event_packet = dv.DepthEventPacket()
        depth_event_packet.elements.append(dv.DepthEvent(100, 1, 1, True, 10000))
        depth_event_packet.elements.append(dv.DepthEvent(100100, 10, 11, False, 20000))
        depth_event_array = depth_event_packet.numpy()
        self.assertTrue(isinstance(depth_event_array, np.ndarray))
        self.assertEqual(depth_event_array["timestamp"][0], 100)
        self.assertEqual(depth_event_array["timestamp"][1], 100100)

        self.assertEqual(depth_event_array["x"][0], 1)
        self.assertEqual(depth_event_array["x"][1], 10)

        self.assertEqual(depth_event_array["y"][0], 1)
        self.assertEqual(depth_event_array["y"][1], 11)

        self.assertEqual(depth_event_array["polarity"][0], 1)
        self.assertEqual(depth_event_array["polarity"][1], 0)

        self.assertEqual(depth_event_array["depth"][0], 10000)
        self.assertEqual(depth_event_array["depth"][1], 20000)

    def test_depth_event_store(self):
        depth_event_store = dv.DepthEventStore()
        depth_event_store.push_back((1100, 1, 0, True, 10000))
        depth_event_store.push_back((2100, 0, 1, False, 20000))
        depth_event_store.push_back((3100, 1, 0, True, 30000))
        depth_event_store.push_back((4100, 0, 1, False, 40000))
        depth_event_store.push_back((5100, 1, 0, True, 50000))

        self.assertEqual(depth_event_store.size(), 5)
        index = 1
        for event in depth_event_store:
            self.assertEqual(event.timestamp(), (index * 1000) + 100)
            self.assertEqual(event.depth(), (index * 10000))
            self.assertEqual(event.polarity(), (index % 2) == 1)
            self.assertEqual(event.x(), index % 2)
            self.assertEqual(event.y(), 1 - (index % 2))
            index += 1

        np_depth = depth_event_store.numpy()
        self.assertEqual(np_depth["timestamp"][0], 1100)
        self.assertEqual(np_depth["timestamp"][1], 2100)
        self.assertEqual(np_depth["timestamp"][2], 3100)
        self.assertEqual(np_depth["timestamp"][3], 4100)
        self.assertEqual(np_depth["timestamp"][4], 5100)

        self.assertEqual(np_depth["x"][0], 1)
        self.assertEqual(np_depth["x"][1], 0)
        self.assertEqual(np_depth["x"][2], 1)
        self.assertEqual(np_depth["x"][3], 0)
        self.assertEqual(np_depth["x"][4], 1)

        self.assertEqual(np_depth["y"][0], 0)
        self.assertEqual(np_depth["y"][1], 1)
        self.assertEqual(np_depth["y"][2], 0)
        self.assertEqual(np_depth["y"][3], 1)
        self.assertEqual(np_depth["y"][4], 0)

        self.assertEqual(np_depth["polarity"][0], True)
        self.assertEqual(np_depth["polarity"][1], False)
        self.assertEqual(np_depth["polarity"][2], True)
        self.assertEqual(np_depth["polarity"][3], False)
        self.assertEqual(np_depth["polarity"][4], True)

        self.assertEqual(np_depth["depth"][0], 10000)
        self.assertEqual(np_depth["depth"][1], 20000)
        self.assertEqual(np_depth["depth"][2], 30000)
        self.assertEqual(np_depth["depth"][3], 40000)
        self.assertEqual(np_depth["depth"][4], 50000)


if __name__ == '__main__':
    unittest.main()

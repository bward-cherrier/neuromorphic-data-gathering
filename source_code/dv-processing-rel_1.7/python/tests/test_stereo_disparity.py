import dv_processing as dv
import unittest


class StereoMatcherTest(unittest.TestCase):

    @staticmethod
    def read_test_data():
        store = dv.EventStore()
        reader = dv.io.MonoCameraRecording("./data/sample_data.aedat4")
        packet = reader.getNextEventBatch()
        while packet is not None:
            store.add(packet)
            packet = reader.getNextEventBatch()
        return store

    def test_disparity(self):
        resolution = (640, 480)
        matcher = dv.SemiDenseStereoMatcher(resolution, resolution)

        store = StereoMatcherTest.read_test_data()

        left_events = dv.EventStore()
        left_events.add(store)
        left_events.add(store)
        left_events.add(store)
        left_events.add(store)

        right_events = dv.EventStore()
        for event in store:
            right_events.push_back(event.timestamp(), event.x() - 10, event.y(), event.polarity())

        right_events.add(right_events)
        right_events.add(right_events)
        right_events.add(right_events)

        disparity = matcher.computeDisparity(left_events, right_events)
        for event in store:
            disparity_value = disparity[event.y(), event.x()] / 16.0
            self.assertEqual(round(disparity_value), 10.0)

    def test_depth_estimation(self):
        resolution = (640, 480)
        store = StereoMatcherTest.read_test_data()

        left_events = dv.EventStore()
        left_events.add(store)
        left_events.add(store)
        left_events.add(store)
        left_events.add(store)

        right_events = dv.EventStore()
        for event in store:
            right_events.push_back(event.timestamp(), event.x() - 10, event.y(), event.polarity())

        right_events.add(right_events)
        right_events.add(right_events)
        right_events.add(right_events)

        ideal_geometry = dv.camera.CameraGeometry(resolution[0] * 0.5, resolution[0] * 0.5, resolution[0] * 0.5,
                                                  resolution[1] * 0.5, resolution)

        stereo_geometry = dv.camera.StereoGeometry(ideal_geometry, ideal_geometry,
                                                   [1, 0, 0, -0.2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        matcher = dv.SemiDenseStereoMatcher(stereo_geometry)
        disparity = matcher.computeDisparity(left_events, right_events)

        depth_frame = matcher.estimateDepthFrame(disparity)
        depth_events = matcher.estimateDepth(disparity, store)
        # Not to run too long, test on small sample
        for event in depth_events.slice(0, 500):
            index = event.y() * resolution[0] + event.x()
            depth = depth_frame.depth[index]
            # OpenCV 4.8.0 issue: https://gitlab.com/inivation/dv/internal/dv-processing-internal/-/issues/118
            # self.assertEqual(depth, event.depth())


if __name__ == '__main__':
    unittest.main()

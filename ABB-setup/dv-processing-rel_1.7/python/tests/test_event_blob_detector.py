import dv_processing as dv
import unittest
import numpy as np


def preprocess(image):
    image *= 0
    return image


class EventBlobDetectorTests(unittest.TestCase):

    def test_preprocessing(self):
        resolution = (640, 480)

        clusters = [(70, 300), (305, 100), (550, 400)]
        events = dv.EventStore()
        std = 9
        for cluster in clusters:
            events.add(dv.data.generate.normallyDistributedEvents(0, cluster, (std, std), 3000))

        pyramidLevel = 3
        blobDetectorWithPreprocess = dv.features.EventBlobDetector(resolution, pyramidLevel, preprocess)
        blobDetectorNoPreprocess = dv.features.EventBlobDetector(resolution, pyramidLevel)
        roi = (0, 0, resolution[0], resolution[1])
        mask = np.ones(resolution, dtype="uint8") * 255
        blobsWithPreprocess = blobDetectorWithPreprocess.detect(events, roi, mask)
        blobs = blobDetectorNoPreprocess.detect(events, roi, mask)

        self.assertEqual(len(blobs), 3)
        self.assertEqual(len(blobsWithPreprocess), 0)

import unittest

from test_event_store import EventStoreTest
from test_depth_events import DepthEventTests
from test_event_blob_detector import EventBlobDetectorTests
from test_stream_slicer import EventStreamSlicerTest
from test_io import FileIOTest
from test_calibration import CalibrationTests
from test_stereo_disparity import StereoMatcherTest
from test_highlevel import HighLevelTest

if __name__ == '__main__':
    unittest.main()

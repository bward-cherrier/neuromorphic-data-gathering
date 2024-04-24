import unittest
import numpy as np
import dv_processing as dv


def write_file(path, name, resolution):
    config = dv.io.MonoCameraWriter.DAVISConfig(name, resolution)
    config.addPoseStream()
    writer = dv.io.MonoCameraWriter(path, config)
    writer.setPackagingCount(10)

    events = dv.EventStore()
    events.push_back((1000000, 10, 10, False))
    events.push_back((2000000, 10, 10, False))
    events.push_back((3000000, 10, 10, False))
    events.push_back((4000000, 10, 10, False))
    events.push_back((5000000, 10, 10, False))
    writer.writeEvents(events)

    writer.writeImu(dv.IMU(1000000, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10))
    writer.writeImu(dv.IMU(2000000, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10))
    writer.writeImu(dv.IMU(3000000, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10))
    writer.writeImu(dv.IMU(4000000, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10))
    writer.writeImu(dv.IMU(5000000, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10))

    writer.writeTrigger(dv.Trigger(1000000, dv.TriggerType.TIMESTAMP_RESET))
    writer.writeTrigger(dv.Trigger(2000000, dv.TriggerType.APS_FRAME_START))
    writer.writeTrigger(dv.Trigger(3000000, dv.TriggerType.APS_EXPOSURE_END))
    writer.writeTrigger(dv.Trigger(4000000, dv.TriggerType.APS_EXPOSURE_START))
    writer.writeTrigger(dv.Trigger(5000000, dv.TriggerType.EXTERNAL_SIGNAL_PULSE))

    writer.writeFrame(dv.Frame(1000000, np.zeros((resolution[1], resolution[0], 3), np.uint8)))
    writer.writeFrame(dv.Frame(5000000, np.zeros((resolution[1], resolution[0], 3), np.uint8) + 255))

    writer.writePose(dv.Pose(1000000, (0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0), "source", "target"))


class FileIOTest(unittest.TestCase):

    def test_write_read(self):
        write_file("./test-write.aedat4", "test", (100, 100))

        reader = dv.io.MonoCameraRecording("./test-write.aedat4")
        self.assertEqual(reader.getCameraName(), "test")
        self.assertEqual(reader.getTimeRange(), (1000000, 5000000))

        frame1 = reader.getNextFrame()
        self.assertEqual(frame1.timestamp, 1000000)
        frame2 = reader.getNextFrame()
        self.assertEqual(frame2.timestamp, 5000000)
        # Only two frames
        self.assertEqual(reader.getNextFrame(), None)

        imu_batch = reader.getNextImuBatch()
        self.assertEqual(len(imu_batch), 5)
        self.assertEqual(imu_batch[0].timestamp, 1000000)
        self.assertEqual(imu_batch[1].timestamp, 2000000)
        self.assertEqual(imu_batch[2].timestamp, 3000000)
        self.assertEqual(imu_batch[3].timestamp, 4000000)
        self.assertEqual(imu_batch[4].timestamp, 5000000)
        self.assertEqual(reader.getNextImuBatch(), None)

        trigger_batch = reader.getNextTriggerBatch()
        self.assertEqual(len(trigger_batch), 5)
        self.assertEqual(trigger_batch[0].timestamp, 1000000)
        self.assertEqual(trigger_batch[1].timestamp, 2000000)
        self.assertEqual(trigger_batch[2].timestamp, 3000000)
        self.assertEqual(trigger_batch[3].timestamp, 4000000)
        self.assertEqual(trigger_batch[4].timestamp, 5000000)
        self.assertEqual(reader.getNextTriggerBatch(), None)

        events = reader.getNextEventBatch()
        self.assertEqual(len(events), 5)
        self.assertEqual(events.getLowestTime(), 1000000)
        self.assertEqual(events.getHighestTime(), 5000000)
        self.assertEqual(reader.getNextEventBatch(), None)

        pose = reader.getNextPose("poses")
        self.assertTrue(reader.isStreamOfPoseType("poses"))
        self.assertFalse(reader.isStreamOfEventType("poses"))
        self.assertEqual(pose.timestamp, 1000000)
        self.assertAlmostEqual(pose.rotation[0], 1.0)
        self.assertAlmostEqual(pose.rotation[1], 0.0)
        self.assertAlmostEqual(pose.rotation[2], 0.0)
        self.assertAlmostEqual(pose.rotation[3], 0.0)
        self.assertAlmostEqual(pose.translation[0], 0.1)
        self.assertAlmostEqual(pose.translation[1], 0.2)
        self.assertAlmostEqual(pose.translation[2], 0.3)
        self.assertEqual(pose.referenceFrame, "source")
        self.assertEqual(pose.targetFrame, "target")
        self.assertEqual(reader.getNextPose("poses"), None)


if __name__ == '__main__':
    unittest.main()

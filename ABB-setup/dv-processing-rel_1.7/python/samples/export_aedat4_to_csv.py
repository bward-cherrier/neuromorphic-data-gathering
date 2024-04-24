import dv_processing as dv
import argparse
import pathlib
import cv2
import numpy as np
import pandas as pd

parser = argparse.ArgumentParser(description='Save aedat4 data to csv.')

parser.add_argument('-f,--file',
                    dest='file',
                    type=str,
                    required=True,
                    metavar='path/to/file',
                    help='Path to an AEDAT4 file')

args = parser.parse_args()

file = pathlib.Path(args.file)
file_parent = file.parent
file_stem = file.stem

# Open the recording file
recording = dv.io.MonoCameraRecording(args.file)
if recording.isFrameStreamAvailable():
    frames_path = file_parent / (file_stem + "_frames")
    if not frames_path.is_dir():
        frames_path.mkdir(parents=True)
    print("Frames will be saved under %s" % frames_path)

    count_frames = 0
    print("Saving Frames...")
    while True:
        frame = recording.getNextFrame()
        if frame is None:
            break
        cv2.imwrite(str(frames_path / (str(frame.timestamp) + '.png')), frame.image)
        count_frames += 1
    print("Saved %d frames." % count_frames)

if recording.isEventStreamAvailable():
    events_path = file_parent / (file_stem + "_events.csv")
    print("Events will be saved under %s" % events_path)

    print("Saving Events...")
    events_packets = []
    while True:
        events = recording.getNextEventBatch()
        if events is None:
            break
        events_packets.append(pd.DataFrame(events.numpy()))
    print("Done reading events, saving into CSV...")
    events_pandas = pd.concat(events_packets)
    events_pandas.to_csv(events_path)
    print("Saved %d events." % len(events_pandas))

if recording.isTriggerStreamAvailable():
    triggers_path = file_parent / (file_stem + "_triggers.csv")
    print("Triggers data will be saved under %s" % triggers_path)
    print("Saving Triggers...")
    types = []
    timestamps = []
    while True:
        triggers = recording.getNextTriggerBatch()
        if triggers is None:
            break
        for trigger in triggers:
            types.append(trigger.type)
            timestamps.append(trigger.timestamp)
    print("Done reading triggers, saving into CSV...")
    triggers_pandas = pd.DataFrame({"timestamp": np.array(timestamps), "type": np.array(types)})
    triggers_pandas.to_csv(triggers_path)
    print("Saved %d triggers." % len(triggers_pandas))

if recording.isImuStreamAvailable():
    imus_path = file_parent / (file_stem + "_imus.csv")
    print("IMU data will be saved under %s" % imus_path)
    print("Saving IMU data...")
    temperatures = []
    accelerometerX = []
    accelerometerY = []
    accelerometerZ = []
    gyroscopeX = []
    gyroscopeY = []
    gyroscopeZ = []
    timestamps = []
    while True:
        imus = recording.getNextImuBatch()
        if imus is None:
            break
        for imu in imus:
            temperatures.append(imu.temperature)
            acc = imu.getAccelerations()
            accelerometerX.append(acc[0])
            accelerometerY.append(acc[1])
            accelerometerZ.append(acc[2])
            gyro = imu.getAngularVelocities()
            gyroscopeX.append(gyro[0])
            gyroscopeY.append(gyro[1])
            gyroscopeZ.append(gyro[2])
            timestamps.append(imu.timestamp)
    print("Done reading IMU data, saving into CSV...")
    imus_pandas = pd.DataFrame({
        "timestamp": np.array(timestamps),
        "temperature": np.array(temperatures),
        "accelerometerX": np.array(accelerometerX),
        "accelerometerY": np.array(accelerometerY),
        "accelerometerZ": np.array(accelerometerZ),
        "gyroscopeX": np.array(gyroscopeX),
        "gyroscopeY": np.array(gyroscopeY),
        "gyroscopeZ": np.array(gyroscopeZ)
    })
    imus_pandas.to_csv(imus_path)
    print("Saved %d IMU measurements." % len(imus_pandas))

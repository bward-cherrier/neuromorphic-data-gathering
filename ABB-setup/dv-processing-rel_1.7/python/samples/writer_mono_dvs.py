import dv_processing as dv
import argparse

parser = argparse.ArgumentParser(description='Record data from a single iniVation camera to a file.')
parser.add_argument("-c",
                    "--camera_name",
                    dest='camera_name',
                    default="",
                    type=str,
                    help="Camera name (e.g. DVXplorer_DXA00093). The application will open any supported camera "
                    "if no camera name is provided.")
parser.add_argument("-o",
                    "--output_path",
                    dest='output_path',
                    type=str,
                    required=True,
                    help="Path to an output aedat4 file for writing.")
args = parser.parse_args()

# Open any camera that is discovered in the system
camera = dv.io.CameraCapture(args.camera_name)

# Check whether frames are available
eventsAvailable = camera.isEventStreamAvailable()
framesAvailable = camera.isFrameStreamAvailable()
imuAvailable = camera.isImuStreamAvailable()
triggersAvailable = camera.isTriggerStreamAvailable()

try:
    # Open a file to write, will allocate streams for all available data types
    writer = dv.io.MonoCameraWriter(args.output_path, camera)

    print("Start recording")
    while camera.isConnected():
        if eventsAvailable:
            # Get Events
            events = camera.getNextEventBatch()
            # Write Events
            if events is not None:
                writer.writeEvents(events, streamName='events')

        if framesAvailable:
            # Get Frame
            frame = camera.getNextFrame()
            # Write Frame
            if frame is not None:
                writer.writeFrame(frame, streamName='frames')

        if imuAvailable:
            # Get IMU data
            imus = camera.getNextImuBatch()
            # Write IMU data
            if imus is not None:
                writer.writeImuPacket(imus, streamName='imu')

        if triggersAvailable:
            # Get trigger data
            triggers = camera.getNextTriggerBatch()
            # Write trigger data
            if triggers is not None:
                writer.writeTriggerPacket(triggers, streamName='triggers')

except KeyboardInterrupt:
    print("Ending recording")
    pass

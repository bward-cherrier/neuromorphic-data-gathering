# -*- coding: utf-8 -*-

import pickle
import time, os, json, math
import threading
from threading import Thread
import  cv2

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController 
from cri.dobot.mg400_controller import MG400Controller

from vsp.video_stream import CvVideoDisplay, CvVideoOutputFile
from vsp.processor import AsyncProcessor, CameraStreamProcessorMP

from core.sensor.tactile_capture_timestamp import CameraStreamProcessorMPTimestamp, CaptureTimestamp
from core.sensor.tactile_sensor_neuro import NeuroTac

class VideoWriterWidget(object):
    def __init__(self, src=0):
        # Create a VideoCapture object
        self.frame_name = str(src)
        self.capture = cv2.VideoCapture(src)

        # Default resolutions of the frame are obtained (system dependent)
        self.frame_width = int(self.capture.get(3))
        self.frame_height = int(self.capture.get(4))

        # # Set up codec and output video settings
        # self.codec = cv2.VideoWriter_fourcc('M','J','P','G')
        # self.output_video = cv2.VideoWriter(self.video_file_name, self.codec, 30, (self.frame_width, self.frame_height))

        self.is_recording = False

        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.read_frames, args=())
        self.thread.daemon = True
        self.thread.start()
        
    def __enter__(self):
        return self
 
    def __exit__(self, type, value, traceback):
        pass

    def set_video_filename(self, video_file_name):
        
        self.video_file = video_file_name
        self.video_file_name = video_file_name + '.avi'
        
        # Set up codec and output video settings
        self.codec = cv2.VideoWriter_fourcc('M','J','P','G')
        self.output_video = cv2.VideoWriter(self.video_file_name, self.codec, 30, (self.frame_width, self.frame_height))
        
 # Start logging data
    def start_logging(self):
        self.is_recording = True
        print('Started recording')

    # Stop logging data
    def stop_logging(self):
        self.is_recording = False
        print('Stopped recording')  
    
    def read_frames(self) -> None:       
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()

    def show_frame(self):
        # Display frames in main program
        if self.status:
            cv2.imshow(self.frame_name, self.frame)
            
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            self.output_video.release()
            cv2.destroyAllWindows()
            exit(1)

    def save_frame(self):
        # Save obtained frame into video output file
        self.output_video.write(self.frame)
    
    def get_frames(self):
        # Create another thread to show/save frames
        while self.is_recording == True:       
            try:
                self.show_frame()
                self.save_frame()
            except AttributeError:
                pass
        # self.capture.release()
        # self.output_video.release()
        # cv2.destroyAllWindows()
        return

def make_meta(robot_tcp = [0, 0, 119.5, 0, 0, 70],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [300, 0, -50, 0, 0, 0], # (300,0,119.5,0,0,70) on online mode
              work_frame = [283.2, 15.9, -190.8, 0, 0, 0], # [283.2, 15.9, -200.8, 0, 0, 0]
              linear_speed = 20,
              angular_speed = 10,
              slide_speeds = [1.0]  ,  # [0.3, 0.5, 1.0]  
              slide_depths = [2,3,4,5],  # [2,3,4,5,6]
              slide_directions = ['right','left','down','up','upright','upleft','downright','downleft','back'], # ['right','left','down','up','upright','upleft','downright','downleft']
              n_runs = 1,
              tip = 'IncUndExt_v1',
              sensor = 'NeuroTac_DVXplorer' # 'NeuroTac_DVXplorer', 'NeuroTac_eDVS', 'Neurotac_DAVIS240'
              ):

    meta = locals().copy()
    return meta

# UR5 Controller
def make_robot():
    return AsyncRobot(SyncRobot(MG400Controller()))

# Camera recording inside the tactip
def make_sensor_in():
    return NeuroTac(save_events_video=True, save_acc_video=False, display=False)    


# # Camera recording outside the tactip     
# def make_sensor_out(video_dir, device_path="/dev/video0", display=CvVideoDisplay(name='tactip_out'), frame_size=(640, 480), fps=60, cut=(0, 0)):
#     return AsyncProcessor(CameraStreamProcessorMPTimestamp(
#         camera=CaptureTimestamp(device_path=device_path, frame_size=frame_size, num_buffers=1, cut=cut, is_color=True),
#         display=display,
#         writer=CvVideoOutputFile(filename=os.path.join(video_dir, 'tactip_out.mp4'), fps=fps,
#                                  is_color=True, fourcc_code='mp4v', frame_size=(640, 480))
#     ))

def make_sensor_out():
    source = '/dev/video0'
    return VideoWriterWidget(source)
    
    
def collect(collect_dir, video_dir, events_dir, robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed, \
    slide_depths, slide_speeds, slide_directions, n_runs, **kwargs):

    ts_data_out = os.path.join(collect_dir, 'timestamps_tactip_out.pkl')
    ts_data_in = os.path.join(collect_dir, 'timestamps_tactip_in.pkl')

    with make_sensor_in() as sensor_in, make_sensor_out() as sensor_out, \
    make_robot() as robot:

        # Set robot parameters
        robot.tcp = robot_tcp
        robot.speed = linear_speed

        # Move home
        _ = robot.move_linear(home_pose)

         # Start sensor
        # sensor_out.process(num_frames=5)

         # Move to origin of work frame
        print("Moving to origin of work frame ...")
        _ = robot.move_linear(work_frame) 

        ts_dict_out = {}
        ts_dict_in = {}
        
        for r in range(n_runs):
            for speed in slide_speeds:
                for depth in slide_depths:
                    for direction in slide_directions:
                        
                        # Set output files
                        events_on_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_on')
                        events_off_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_off')
                        events_stream_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_stream')
                        tactip_out_video = os.path.join(video_dir, 'tactip_out_d'+ str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r))
                        events_video = os.path.join(video_dir,'event_stream_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
                        acc_video = os.path.join(video_dir,'accumulator_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')        

                        sensor_in.reset_variables()
                        sensor_in.set_filenames(events_on_file = events_on_file, events_off_file = events_off_file,events_stream_file = events_stream_file, events_video_file = events_video, acc_video_file = acc_video)
                        sensor_out.set_video_filename(video_file_name = tactip_out_video)
                        
                        # Set robot positions
                        retract = 3
                        slide = 15
                        ur5_positions = [[0,0,0,0,0,0],[0,0,-10-depth,0,0,0],[0,0,-10-depth+retract,0,0,0],[0,0,0,0,0,0]]
                        if direction == 'up':
                            ur5_positions[2] = [0,slide,-10-depth,0,0,0]
                        elif direction == 'down':
                            ur5_positions[2] = [0,-slide,-10-depth,0,0,0]
                        elif direction == 'right':
                            ur5_positions[2] = [slide,0,-10-depth,0,0,0]
                        elif direction == 'left':
                            ur5_positions[2] = [-slide,0,-10-depth,0,0,0]
                        elif direction == 'upright':
                            ur5_positions[2] = [slide/math.sqrt(2),slide/math.sqrt(2),-10-depth,0,0,0]
                        elif direction == 'upleft':
                            ur5_positions[2] = [-slide/math.sqrt(2),slide/math.sqrt(2),-10-depth,0,0,0]
                        elif direction == 'downright':
                            ur5_positions[2] = [slide/math.sqrt(2),-slide/math.sqrt(2),-10-depth,0,0,0]
                        elif direction == 'downleft':
                            ur5_positions[2] = [-slide/math.sqrt(2),-slide/math.sqrt(2),-10-depth,0,0,0]
                        elif direction == 'back':
                            ur5_positions[2] = [0,0,-10-depth+retract,0,0,0]
                        ur5_positions = [list(map(sum,zip(p,work_frame))) for p in ur5_positions]

                        # Move to starting point
                        print("Moving to " + str(ur5_positions[0]))
                        _ = robot.move_linear(ur5_positions[0])
                        print('depth: ' + str(depth) + ', speed: ' + str(speed) + ', run: ' + str(r) + ', direction: ' + str(direction))

                        # Check depth is safe
                        if depth > 10:
                            raise Exception("Risk of damaging tactip...quitting")

                        # Contact perspex
                        robot.speed = 4
                        _ = robot.move_linear(ur5_positions[1])  # Contact object

                        # Start sensors
                        # sensor_out.async_process(outfile=tactip_out_video, num_frames=5000)  # Start exterior cam rec
                        # # sensor_out.async_process(outfile=None, num_frames=5000)
                        sensor_out.start_logging()
                        t_out = threading.Thread(target=sensor_out.get_frames, args = ())
                        t_out.start()
                        
                        sensor_in.start_logging()    
                        t_in = threading.Thread(target=sensor_in.get_events, args = ())
                        t_in.start()
                                    
                        time.sleep(1)

                        # Start moving back
                        robot.speed = speed
                        _ = robot.move_linear(ur5_positions[2])
                        
                        # Stop recording
                        sensor_in.stop_logging()
                        t_in.join()
                        
                        sensor_out.stop_logging()
                        t_out.join()
                        # sensor_out.async_cancel()

                        # # Write videos and timestamps to file
                        # _, ts_out = sensor_out.async_result()
                                # Save data

                        sensor_in.value_cleanup()
                        sensor_in.save_events_stream()
                        # sensor_in.save_events_on()
                        # sensor_in.save_events_off()

                        # Update timestamps 
                        # ts_dict_out.update({'tactip_out_d'+ str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r): ts_out})
                        # ts_dict_in.update({'tactip_in_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r): ts_in})
                        # ts_out = None
                        # # ts_in = None        

                        # Move back from perspex
                        robot.speed = linear_speed
                        _ = robot.move_linear(ur5_positions[3])

        # Return home
        print("Moving home ...")
        robot.speed = linear_speed
        robot.move_linear(home_pose)  # Move home

        # Save timestamp data
        # with open(ts_data_out, 'wb') as t:
        #     pickle.dump(ts_dict_out, t)
        # with open(ts_data_in, 'wb') as t:
        #     pickle.dump(ts_dict_in, t)
        
def main():

    # Directories
    collect_dir_name = os.path.join('horizontal_shear', 'horizontal_shear_' + time.strftime('%m%d%H%M'))
    collect_dir = os.path.join(os.environ['DATAPATH'], 'NeuroTac_DVXplorer', collect_dir_name)
    video_dir = os.path.join(collect_dir, 'videos')
    events_dir = os.path.join(collect_dir, 'events')
    
    os.makedirs(collect_dir, exist_ok=True)
    os.makedirs(video_dir, exist_ok=True)
    os.makedirs(events_dir, exist_ok=True)

    # Make and save meta data and dataframe
    meta = make_meta() 
    with open(os.path.join(collect_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f) 

    # Collect data
    collect(collect_dir,video_dir,events_dir,**meta)

if __name__ == '__main__':
    main()

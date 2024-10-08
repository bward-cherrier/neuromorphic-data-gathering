# -*- coding: utf-8 -*-
"""Test script for v4l2 camera interface.
"""

import time, os, json, pickle

import cv2
from threading import Thread

import threading
from pathlib import Path

from core.sensor.tactile_sensor_neuro import NeuroTac

class VideoWriterWidget(object):
    def __init__(self, src=0):
        # Create a VideoCapture object
        self.src = src
        self.frame_name = str(src)
        self.capture = cv2.VideoCapture(src)

        # Default resolutions of the frame are obtained (system dependent)
        self.frame_width = int(self.capture.get(3))
        self.frame_height = int(self.capture.get(4))

        # # Set up codec and output video settings
        # self.codec = cv2.VideoWriter_fourcc('M','J','P','G')
        # self.output_video = cv2.VideoWriter(self.video_file_name, self.codec, 30, (self.frame_width, self.frame_height))

        self.is_recording = False
        self.timestamps = []

        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.read_frames, args=())
        self.thread.daemon = True
        self.thread.start()
        
    def __enter__(self):
        return self
 
    def __exit__(self, type, value, traceback):
        pass

    def reset_variables(self):
        self.timestamps = []
        
    def set_video_filename(self, video_file_name):
        
        self.video_file = video_file_name
        self.video_file_name = video_file_name + '.avi'
        
        # Set up codec and output video settings
        self.codec = cv2.VideoWriter_fourcc('M','J','P','G')
        self.output_video = cv2.VideoWriter(self.video_file_name, self.codec, 30, (self.frame_width, self.frame_height))
        
 # Start logging data
    def start_logging(self):
        self.is_recording = True
        print('Recording from camera')
        self.start_timestamp = time.time()
        # self.capture = cv2.VideoCapture(self.src)


    # Stop logging data
    def stop_logging(self):
        self.is_recording = False
        print('Stopped recording from camera')  
    
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
        self.timestamps.append(time.time())
    
    def get_frames(self):
        # Create another thread to show/save frames
        while self.is_recording == True:       
            try:
                # self.show_frame()
                self.save_frame()
            except AttributeError:
                pass
        # self.capture.release()
        self.output_video.release()
        cv2.destroyAllWindows()
        return
    
    def get_timestamps(self):
        return [(ts - self.start_timestamp)*1000 for ts in self.timestamps]


def make_meta(robot_tcp = [0, 0, 115, 0, 0, 70],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [109.1, -456.0, 340.0, 180, 0, -160],
              work_frame = [383, -257, 192.5, 180, 0, -160], # (383, -257, 280, 180, 0, 0), (383, -257, 180, 180, 0, -135) for old tactip
              linear_speed = 20,
              angular_speed = 10,
              slide_speeds = [1.0]  ,  # [0.3, 0.5, 1.0]  
              slide_depths = [2],  # [2,3,4,5,6]
              slide_directions = ['right','left','down','up','upright','upleft','downright','downleft','back'], # ['right','left','down','up','upright','upleft','downright','downleft']
              n_runs = 3,
              tip = 'IncUndExt_v1',
              sensor = 'NeuroTac_DVXplorer' # 'NeuroTac_DVXplorer', 'NeuroTac_eDVS', 'Neurotac_DAVIS240'
              ):

    meta = locals().copy()
    return meta

def make_sensor_in():
    return NeuroTac(save_events_video=True, save_acc_video=False, display=False)    
           
def make_sensor_out():
    source = '/dev/video0'
    return VideoWriterWidget(source)
    

def collect(collect_dir, video_dir, events_dir, robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed, \
    slide_depths, slide_speeds, slide_directions, n_runs, **kwargs):
    with make_sensor_in() as sensor_in, make_sensor_out() as sensor_out: 

        # sensor_out.process(num_frames=5)
        time.sleep(1)

        start = time.time()

        for r in range(n_runs):
            
            frames = 60000
            # sensor_out.async_process(outfile=os.path.join(video_dir, 'tactip_out.mp4'), num_frames=frames)
            # time.sleep(1)
            sensor_in.reset_variables()
            sensor_out.reset_variables()
            
            depth = slide_depths[0]
            speed = slide_speeds[0]
            direction = slide_directions[0]
            
            events_on_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_on')
            events_off_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_off')
            events_video_file = os.path.join(video_dir,'event_stream_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
            acc_video_file = os.path.join(video_dir,'accumulator_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
            tactip_out_video = os.path.join(video_dir, 'tactip_out_d'+ str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r))

            sensor_in.set_filenames(events_on_file=events_on_file, events_off_file=events_off_file, events_video_file=events_video_file, acc_video_file=acc_video_file)
            sensor_out.set_video_filename(video_file_name = tactip_out_video)

            sensor_out.start_logging()
            t_out = threading.Thread(target=sensor_out.get_frames, args = ())
            t_out.start()
            
            sensor_in.start_logging()
            t = threading.Thread(target=sensor_in.get_events, args = ())
            t.start()
            # sensor_FT.async_read(num_frames = 5000)
            
            time.sleep(2)
            
            # Stop recording
            sensor_in.stop_logging()
            t.join()
            sensor_out.stop_logging()
            t_out.join()

            sensor_in.value_cleanup()
            # Stop external camera
            # sensor_out.async_cancel()
            
            # Stop external camera
            # frames_in, ts_in = sensor_in.async_result()
            # frames_out, ts_out = sensor_out.async_result()
            
            # events_on_file = os.path.join(events_dir, 'events_on')
            # events_off_file = os.path.join(events_dir, 'events_off')
                       
            sensor_in.save_events_on(events_on_file)
            sensor_in.save_events_off(events_off_file)
            
        end = time.time()
        
        
        # sensor_in.close()
        # # sensor_out.close()
        
        
def main():      
        
    collect_dir_name = os.path.join('camera_tests', 'camera_tests_' + time.strftime('%m%d%H%M'))
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

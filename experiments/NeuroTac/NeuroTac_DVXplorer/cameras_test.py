# -*- coding: utf-8 -*-
"""Test script for v4l2 camera interface.
"""

import time, os, json, pickle

import cv2

import threading
from pathlib import Path

from vsp.video_stream import CvVideoDisplay, CvVideoInputFile, CvVideoOutputFile
from vsp.v4l2_camera import V4l2VideoCamera

from vsp.video_stream import CvVideoDisplay, CvVideoOutputFile
from vsp.processor import AsyncProcessor

from core.sensor.tactile_capture_timestamp import CaptureTimestampOnline, CameraStreamProcessorMPTimestamp, CaptureTimestamp
from core.sensor.tactile_sensor_neuro import NeuroTac


def make_meta(robot_tcp = [0, 0, 115, 0, 0, 70],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [109.1, -456.0, 340.0, 180, 0, -160],
              work_frame = [383, -257, 192.5, 180, 0, -160], # (383, -257, 280, 180, 0, 0), (383, -257, 180, 180, 0, -135) for old tactip
              linear_speed = 20,
              angular_speed = 10,
              slide_speeds = [1.0]  ,  # [0.3, 0.5, 1.0]  
              slide_depths = [2],  # [2,3,4,5,6]
              slide_directions = ['right','left','down','up','upright','upleft','downright','downleft','back'], # ['right','left','down','up','upright','upleft','downright','downleft']
              n_runs = 1,
              tip = 'IncUndExt_v1',
              sensor = 'NeuroTac_DVXplorer' # 'NeuroTac_DVXplorer', 'NeuroTac_eDVS', 'Neurotac_DAVIS240'
              ):

    meta = locals().copy()
    return meta

def make_sensor_in():
    return NeuroTac(save_events_video=True, save_acc_video=True, display=True)    
           
def make_sensor_out(video_dir, device_path="/dev/video0", display=CvVideoDisplay(name='neurotac_out'), frame_size=(640, 480), fps=60, cut=(0, 0)):
    return AsyncProcessor(CameraStreamProcessorMPTimestamp(
        camera=CaptureTimestamp(device_path=device_path, frame_size=frame_size, num_buffers=1, cut=cut, is_color=True),
        display=display,
        writer=CvVideoOutputFile(filename=os.path.join(video_dir, 'tactip_out.mp4'), fps=fps,
                                 is_color=True, fourcc_code='mp4v', frame_size=(640, 480))
    ))

def collect(collect_dir, video_dir, events_dir, robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed, \
    slide_depths, slide_speeds, slide_directions, n_runs, **kwargs):
    with make_sensor_in() as sensor_in, make_sensor_out(video_dir, cut=None) as sensor_out: 

        # sensor_out.process(num_frames=5)
        time.sleep(1)

        start = time.time()

        for r in range(n_runs):
            
            frames = 60000
            sensor_out.async_process(outfile=os.path.join(video_dir, 'tactip_out.mp4'), num_frames=frames)
            # time.sleep(1)
            sensor_in.reset_variables()
            
            depth = slide_depths[0]
            speed = slide_speeds[0]
            direction = slide_directions[0]
            
            events_on_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_on')
            events_off_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_off')
            events_video_file = os.path.join(video_dir,'event_stream_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
            acc_video_file = os.path.join(video_dir,'accumulator_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
                    
            sensor_in.set_filenames(events_on_file=events_on_file, events_off_file=events_off_file, events_video_file=events_video_file, acc_video_file=acc_video_file)
        
            sensor_in.start_logging()
            
            t = threading.Thread(target=sensor_in.get_events, args = ())
            t.start()
            # sensor_FT.async_read(num_frames = 5000)
            
            time.sleep(100)
            
            # Stop recording
            sensor_in.stop_logging()
            t.join()

            sensor_in.value_cleanup()
            # Stop external camera
            sensor_out.async_cancel()
            
            # Stop external camera
            # frames_in, ts_in = sensor_in.async_result()
            frames_out, ts_out = sensor_out.async_result()
            
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

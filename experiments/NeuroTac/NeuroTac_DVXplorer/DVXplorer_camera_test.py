# -*- coding: utf-8 -*-
"""Test script for v4l2 camera interface.
"""

import time, os, json, pickle

import cv2
import threading
from pathlib import Path

from vsp.video_stream import CvVideoDisplay, CvVideoInputFile, CvVideoOutputFile
from vsp.v4l2_camera import V4l2VideoCamera
from vsp.processor import AsyncProcessor

from core.sensor.tactile_capture_timestamp import CaptureTimestampOnline, CameraStreamProcessorMPTimestamp, CaptureTimestamp

# os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.fspath(
#     Path(PyQt5.__file__).resolve().parent / "Qt5" / "plugins"
# )

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

def make_sensor():
    return NeuroTac(save_events_video=True, save_acc_video=False, display=True)    
           

def collect(collect_dir, video_dir, events_dir, robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed, \
    slide_depths, slide_speeds, slide_directions, n_runs, **kwargs):
    with make_sensor() as sensor: 

        # sensor_out.process(num_frames=5)
        time.sleep(1)

        start = time.time()

        for r in range(n_runs):
            
            frames = 60000

            # time.sleep(1)
            sensor.reset_variables()
            
            depth = slide_depths[0]
            speed = slide_speeds[0]
            direction = slide_directions[0]
            
            events_on_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_on')
            events_off_file = os.path.join(events_dir,'d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'_events_off')
            events_video_file = os.path.join(video_dir,'event_stream_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
            acc_video_file = os.path.join(video_dir,'accumulator_d' + str(depth) + '_s' + str(speed) + '_dir_' + direction + '_r' + str(r)+'.mp4')
                    
            sensor.set_filenames(events_on_file=events_on_file, events_off_file=events_off_file, events_video_file=events_video_file, acc_video_file=acc_video_file)
        
            sensor.start_logging()
            
            t = threading.Thread(target=sensor.get_events, args = ())
            t.start()
            # sensor_FT.async_read(num_frames = 5000)
            
            time.sleep(10)
            
            # Stop recording
            sensor.stop_logging()
            t.join()

            sensor.value_cleanup()
                       
            sensor.save_events_on(events_on_file)
            sensor.save_events_off(events_off_file)
            
        end = time.time()
        
        # sensor.close()
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

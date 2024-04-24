# -*- coding: utf-8 -*-
import os, time, json
import numpy as np
import pickle
import threading 

from core.sensor.tactile_sensor_neuro import NeuroTac

sensor_type = 'NeuroTac_DVXplorer' # NeuroTac version. Options: 'NeuroTac_DVXplorer', 'NeuroTac_DAVIS240', 'NeuroTac_eDVS' 

def make_meta (robot_tcp = [0, 0, 115, 0, 0, 0], # Size of the TacTip (tcp = tool center point)
              base_frame = [0, 0, 0, 0, 0, 0], # Origin of the base frame is at centre of the ABB robot base
              home_pose = [400, 0, 340, 180, 0, 180], # Starting point of the robot when switched on
              work_frame = [429, 0, 200, 180, 0, 180], # Starting point and reference frame for this experiment 
              linear_speed = 10, # Robot's speed for linear movements
              angular_speed = 20, # Robot's speed for rotation movements
              tap_move = [[0, 0, 5, 0, 0, 0], [0, 0, 0, 0, 0, 0]],   # Sequence of positions for the tap relative to the robot's current pose (positive z goes down)
              n_trials = 2, # Number of taps at each pose
              sensor = sensor_type # defines the NeuroTac version used: 'NeuroTac_DVXplorer', 'NeuroTac_eDVS', 'Neurotac_DAVIS240'
              ):
  meta = locals().copy()
  return meta

def make_sensor():  
  return NeuroTac(save_events_video=True, save_acc_video=False, display=False)

def collect_data(collect_dir, video_dir, events_dir, tap_move, home_pose, base_frame, work_frame,robot_tcp,linear_speed,angular_speed,n_trials,**kwargs):
  with make_sensor() as sensor:

    for trial_idx in range (n_trials):

      print('#### Trial ' + str(trial_idx+1) + ' / ' + str(n_trials) + ' ####')

      sensor.reset_variables()
      events_on_file = os.path.join(events_dir,'trial_' +str(trial_idx) + '_events_on')
      events_off_file = os.path.join(events_dir,'trial_' +str(trial_idx) + '_events_off')
      events_video = os.path.join(video_dir,'event_stream_trial_' +str(trial_idx) + '.mp4')   
      acc_video = os.path.join(video_dir,'accumulator_trial_' +str(trial_idx) + '.mp4')                   
      sensor.set_filenames(events_on_file = events_on_file, events_off_file = events_off_file, events_video_file = events_video, acc_video_file = acc_video)

      # Start sensor recording
      sensor.start_logging()
      t = threading.Thread(target=sensor.get_events, args = ())
      t.start()

      time.sleep(5)

      # Stop sensor recording
      sensor.stop_logging()
      t.join()

      # Collate proper timestamp values in ms.
      sensor.value_cleanup()

      # Save data
      sensor.save_events_on()
      sensor.save_events_off()

      print("Saved data")

def main():

  # Directories
  collect_dir_name = os.path.join(os.path.basename(__file__)[:-3], os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M'))
  collect_dir = os.path.join(os.environ['DATAPATH'], sensor_type, collect_dir_name)
  video_dir = os.path.join(collect_dir, 'videos')
  events_dir = os.path.join(collect_dir, 'events')
  
  os.makedirs(collect_dir, exist_ok=True)
  os.makedirs(video_dir, exist_ok=True)
  os.makedirs(events_dir, exist_ok=True)

  meta = make_meta() 
  with open(os.path.join(collect_dir, 'meta.json'), 'w') as f: 
    json.dump(meta, f)   
  
  # Collect data
  collect_data(collect_dir, video_dir, events_dir, **meta)

if __name__ == '__main__':
    main()

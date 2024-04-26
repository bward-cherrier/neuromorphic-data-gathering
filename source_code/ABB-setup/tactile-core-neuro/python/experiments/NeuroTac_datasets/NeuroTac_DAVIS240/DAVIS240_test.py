# -*- coding: utf-8 -*-
import os, time, json
import numpy as np
import pickle
import threading 

from core.sensor.tactile_sensor_neuro import NeuroTac_DAVIS240

def make_meta(meta_file, 
              sensor = 'DAVIS240', # Sensor type: DAVIS240, eDVS or DVXplorer           
              robot_tcp = [0, 0, 75, 0, 0, 0], # Size of the TacTip (tcp = tool center point)
              base_frame = [0, 0, 0, 0, 0, 0], # Origin of the base frame is at centre of the ABB robot base
              home_pose = [400, 0, 340, 180, 0, 180], # Starting point of the robot when switched on
              work_frame = [366, -190, 112, 180, 0, 180], # Starting point and reference frame for this experiment 
              linear_speed = 10, # Robot's speed for linear movements
              angular_speed = 20, # Robot's speed for rotation movements
              tap_move = [[0, 0, 5, 0, 0, 0], [0, 0, 0, 0, 0, 0]],   # Sequence of positions for the tap relative to the robot's current pose (positive z goes down)
              obj_poses = [[0, 0, 0, 0, 0, ang] for ang in range(0,9,10)], # Position of the different objects/poses relative to the work frame origin
              n_trials = 3, # Number of taps at each pose
              ):
  data_dir = os.path.dirname(meta_file)

  meta = locals().copy()
  del meta['data_dir']
  return meta

def make_sensor():  
  return NeuroTac_DAVIS240(port=7777)

def collect_data(data_dir, tap_move, obj_poses, home_pose, base_frame, work_frame,robot_tcp,linear_speed,angular_speed,n_trials,**kwargs):
  with make_sensor() as sensor:

    starttimes = []
    for trial_idx in range (n_trials):
      pose_idx = 0
      n_poses = len(obj_poses)
      for pose_idx in range(n_poses):

        print('#### Pose ' + str(pose_idx+1) + ' / ' + str(n_poses) + ' Trial ' + str(trial_idx+1) + ' / ' + str(n_trials) + ' ####')

        sensor.reset_variables()

        # Start sensor recording
        sensor.start_logging()
        t = threading.Thread(target=sensor.get_pixel_events, args = ())
        t.start()

        time.sleep(5)

        # Stop sensor recording
        sensor.stop_logging()
        t.join()

        # Collate proper timestamp values in ms.
        sensor.convert_events_to_milliseconds()
        starttimes.append(sensor.get_starttime())

        # Save data
        events_on_file = os.path.join(data_dir,'trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '_events_on.pickle')
        events_off_file = os.path.join(data_dir,'trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '_events_off.pickle')
        sensor.save_events_on(events_on_file)
        sensor.save_events_off(events_off_file)

        print("Saved data")
        pose_idx +=1
        
    starttimes_file = os.path.join(data_dir,'starting_timestamps.pickle')
    pickle_starttimes = open(starttimes_file, 'wb')
    pickle.dump(starttimes, pickle_starttimes)
    pickle_starttimes.close()

def main():

  # Make and save metadata
  data_dir = os.path.join(os.environ['DATAPATH'], 'NeuroTac_DAVIS240', os.path.basename(__file__)[:-3], os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'))
  meta_file = os.path.join(data_dir, 'meta.json')
  meta = make_meta(meta_file) 
  os.makedirs(os.path.dirname(meta_file),exist_ok=True)
  with open(meta_file, 'w') as f: 
    json.dump(meta, f)   
  
  # Collect data
  collect_data(data_dir,**meta)

if __name__ == '__main__':
    main()

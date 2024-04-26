# -*- coding: utf-8 -*-
    
import os, time, json, shutil
import numpy as np
import pandas as pd
import pickle
import threading 

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController
from core.sensor.tactile_sensor_eDVS_gui import TacTip_edvs

# from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
# from vsp.processor import CameraStreamProcessorMT, AsyncProcessor

def make_meta(meta_file,              
              robot_tcp = [0, 0, 75, 0, 0, 0],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [400, 0, 300, 180, 0, 180],
              work_frame = [366, -14, 79.8, 180, 0, 180], 
              linear_speed = 10,
              angular_speed = 10,
              num_frames = 1,
              tap_move = [[0, 0, 4, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
              poses_rng = [[-5, 0, -3, -15, -15, -45], [5, 0, 0, 15, 15, 45]],
              obj_poses = [[0, 0, 0, 0, 0, ang] for ang in range(0,179,10)],
              num_poses = 18,
              num_trials = 20, 
              ):
  data_dir = os.path.dirname(meta_file)

  video_dir = os.path.join(data_dir, 'videos')
  video_df_file = os.path.join(data_dir, 'targets_video.csv')

  meta = locals().copy()
  del meta['data_dir']
  return meta

def make_robot():
  return AsyncRobot(SyncRobot(ABBController(ip='192.168.125.1')))

def make_sensor():  
  return TacTip_edvs()

def collect_data(data_dir, tap_move, obj_poses, home_pose, base_frame, work_frame,robot_tcp,linear_speed,angular_speed,num_trials,**kwargs):
  with make_robot() as robot, make_sensor() as sensor:

    for r in range (num_trials):

      robot.tcp = robot_tcp
      
      # move to home position
      print("Moving to home position ...")
      robot.coord_frame = base_frame
      robot.linear_speed = 50
      robot.move_linear(home_pose)

      # Set TCP, linear speed,  angular speed and coordinate frame
      robot.linear_speed = linear_speed
      robot.angular_speed = angular_speed

      # Display robot info
      print("Robot info: {}".format(robot.info))

      # Display initial pose in work frame
      print("Initial pose in work frame: {}".format(robot.pose))
      
      # Move to origin of work frame
      print("Moving to origin of work frame ...")
      robot.coord_frame = work_frame
      robot.move_linear((0, 0, 0, 0, 0, 0))
      obj_idx = 0

      for pose in obj_poses:

        sensor.set_variables()

        robot.move_linear(pose)
        # time.sleep(1)

        # Start sensor recording
        sensor.start_logging()
        t = threading.Thread(target=sensor.get_pixel_events, args = ())
        t.start()

        # Tap
        print("Trial Tap " + str(obj_idx+1) + "/" + str(len(obj_poses)))  
        robot.coord_frame = base_frame
        robot.coord_frame = robot.pose
        robot.move_linear(tap_move[0])
        robot.move_linear(tap_move[1])
        robot.coord_frame = work_frame

        # Stop sensor recording
        sensor.stop_logging()
        t.join()
        # print("tap finished")

        # Collate proper timestamp values in ms.
        sensor.value_cleanup()
        # print("value cleaned")

        # Save data
        pickle_out = open(os.path.join(data_dir,'SingleTap_run_' +str(r) +'_orientation_'  + str(obj_idx) + '.pickle'), 'wb')
        pickle.dump(sensor.data, pickle_out)
        pickle_out.close()
        print("saved data")
        obj_idx +=1

      # # Unwind sensor
      print("Unwinding")
      robot.move_linear([sum(x) for x in zip(robot.pose, [0,0,0,0,0,-170])])
      # robot.move_linear([sum(x) for x in zip(robot.pose, [0,0,0,0,0,-170])])

      # Move to home position
      print("Moving to home position ...")
      robot.coord_frame = base_frame
      robot.linear_speed = 50
      robot.move_linear(home_pose)


def main():

  # Make and save metadata
  data_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_NM', 'edgeTap',os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'))
  meta_file = os.path.join(data_dir, 'meta.json')
  meta = make_meta(meta_file) 
  os.makedirs(os.path.dirname(meta_file))
  with open(meta_file, 'w') as f: 
    json.dump(meta, f)   
  
  # Collect data
  collect_data(data_dir,**meta)

  


if __name__ == '__main__':
    main()

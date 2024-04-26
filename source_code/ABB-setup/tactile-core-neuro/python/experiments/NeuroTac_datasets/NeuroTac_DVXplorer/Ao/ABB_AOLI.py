# -*- coding: utf-8 -*-
    
import os, time, json
import numpy as np
import pickle
import threading 

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController
from core.sensor.tactile_sensor_neuro import NeuroTac

from Pyro5.api import Proxy

# from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
# from vsp.processor import CameraStreamProcessorMT, AsyncProcessor

sensor_type = 'NeuroTac_DVXplorer' # NeuroTac version. Options: 'NeuroTac_DVXplorer', 'NeuroTac_DAVIS240', 'NeuroTac_eDVS' 

def make_meta(robot_tcp = [0, 0, 59, 0, 0, 0], # Size of the TacTip (tcp = tool center point)
              base_frame = [0, 0, 0, 0, 0, 0], # Origin of the base frame is at centre of the ABB robot base0
              home_pose = [400, 0, 240, 180, 0, 180], # Starting point of the robot when switched on
              work_frame = [500, -180, 19.5, 180, 0, 180], # Starting point and reference frame for this experiment 
              linear_speed = 10, # Robot's speed for linear movements
              angular_speed = 20, # Robot's speed for rotation movements
              tap_move = [[0, 0, 1.0, 0, 0, 0], [0, 10, 1.0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],   # Sequence of positions for the tap relative to the robot's current pose (positive z goes down)
              # obj_poses = [[0, y, 0, 0, 0, 0]for y in range(0, 151, 15)], # Position of the different objects/poses relative to the work frame origin
              obj_poses = [[0, 0, 0, 0, 0, 0], [90, 0, -0.5, 0, 0, 0]],
              n_trials = 100, # Number of taps at each pose
              ):
  meta = locals().copy()
  return meta

def make_robot():
  return AsyncRobot(SyncRobot(ABBController(ip='192.168.125.1')))

def make_sensor():  
  return NeuroTac(save_events_video=True, save_acc_video=False, display=False)

# def make_sensor(service = "neurotac_service_1"):  
#   return Proxy(f"PYRONAME:{service}")
  

def collect_data(collect_dir, video_dir, events_dir, tap_move, obj_poses, home_pose, base_frame, work_frame,robot_tcp,linear_speed,angular_speed,n_trials,**kwargs):
  with make_robot() as robot, make_sensor() as sensor:

    robot.tcp = robot_tcp
    
    # move to home position
    print("Moving to home position ...")
    robot.coord_frame = base_frame
    robot.linear_speed = 80
    robot.move_linear(home_pose)

    # Set TCP, linear speed,  angular speed and coordinate frame
    robot.linear_speed = linear_speed
    robot.angular_speed = angular_speed

    # Display robot info
    print("Robot info: {}".format(robot.info))

    # Display initial pose in work frame
    print("Initial pose in work frame: {}".format(robot.pose))

    for trial_idx in range (n_trials):

      # Move to origin of work frame
      print("Moving to origin of work frame ...")
      robot.linear_speed = 100
      robot.coord_frame = work_frame
      robot.move_linear((0, 0, 0, 0, 0, 0))
      robot.linear_speed = linear_speed
      pose_idx = 0
      # tap_move = [[0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0]]

      for pose_idx in range(len(obj_poses)):

        events_on_file = os.path.join(events_dir,'taps_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '_events_on')
        events_off_file = os.path.join(events_dir,'taps_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '_events_off')
        events_video = os.path.join(video_dir,'event_stream_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) +'.mp4')
        acc_video = os.path.join(video_dir,'accumulator_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) +'.mp4')                    
        sensor.set_filenames(events_on_file = events_on_file, events_off_file = events_off_file, events_video_file = events_video, acc_video_file = acc_video)

        sensor.reset_variables()
        robot.linear_speed = 100
        robot.move_linear(obj_poses[pose_idx])
        robot.linear_speed = linear_speed
        time.sleep(0.5)

        # Tap
        print("Trial " + str(trial_idx+1) + "/" + str(n_trials) + " Pose " + str(pose_idx+1) + "/" + str(len(obj_poses)))  
        robot.coord_frame = base_frame
        robot.coord_frame = robot.pose

        # Start sensor recording
        sensor.start_logging()
        t = threading.Thread(target=sensor.get_events, args = ())
        t.start()

        # start = time.time()
        robot.move_linear(tap_move[0])
        time.sleep(0.1)
        robot.move_linear(tap_move[1])
        # end = time.time()

        # time_taken = end - start

        # Stop sensor recording
        sensor.stop_logging()
        t.join()

        robot.coord_frame = work_frame
        robot.linear_speed = linear_speed

        # Collate proper timestamp values in ms.
        sensor.value_cleanup()

        # Save data
        sensor.save_events_on(events_on_file)
        # sensor.save_events_off(events_off_file)
        print("saved data")
        pose_idx +=1

    # Move to home position
    print("Moving to home position ...")
    robot.coord_frame = base_frame
    robot.linear_speed = 80
    robot.move_linear(home_pose)

  
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

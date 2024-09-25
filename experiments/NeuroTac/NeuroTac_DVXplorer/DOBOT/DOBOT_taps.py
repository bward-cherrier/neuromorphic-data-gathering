# -*- coding: utf-8 -*-

import pickle
import time, os, json, math
import threading
from threading import Thread
import  cv2

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController 
from cri.dobot.mg400_controller import MG400Controller

from core.sensor.tactile_sensor_neuro import NeuroTac

sensor_type = 'NeuroTac_DVXplorer' # NeuroTac version. Options: 'NeuroTac_DVXplorer', 'NeuroTac_DAVIS240', 'NeuroTac_eDVS' 

def make_meta(robot_tcp = [0, 0, 119.5, 0, 0, 70],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [280, 0, -80, 0, 0, 0], 
              work_frame = [233.2, -9.9, -100.5, 0, 0, 0], # old [283.2, 15.9, -200.8, 0, 0, 0] -119.5 difference in z in windows software
              linear_speed = 20,
              angular_speed = 10,
              tap_move = [[0, 0, -4, 0, 0, 0], [0, 0, 0, 0, 0, 0]], 
              obj_poses = [[0, y, 0, 0, 0, 0] for y in range(0, 50, 25)], # Position of the different objects/poses relative to the work frame origin      
              n_trials = 10,
              sensor = sensor_type # 'NeuroTac_DVXplorer', 'NeuroTac_eDVS', 'Neurotac_DAVIS240'
              ):

    meta = locals().copy()
    return meta

# UR5 Controller
def make_robot():
    return AsyncRobot(SyncRobot(MG400Controller()))

def make_sensor():
    return NeuroTac(save_events_video=True, save_acc_video=False, display=False)    
    
    
def collect(collect_dir, video_dir, events_dir, robot_tcp, base_frame, home_pose, work_frame, linear_speed, tap_move, obj_poses, n_trials, **kwargs):

    with make_sensor() as sensor, make_robot() as robot:

        # Set robot parameters
        robot.tcp = robot_tcp
        robot.speed = linear_speed

        # Move home
        _ = robot.move_linear(home_pose)

        #  # Move to origin of work frame
        # print("Moving to origin of work frame ...")
        # _ = robot.move_linear(work_frame) 
        
        for trial_idx in range(n_trials):
          
                # Move to origin of work frame
            print("Moving to origin of work frame ...")
            robot.coord_frame = work_frame
            robot.move_linear((0, 0, 0, 0, 0, 0))
            robot.linear_speed = linear_speed
            pose_idx = 0
      
            for pose_idx in range(len(obj_poses)):
                        
                # Set output files
                sensor.reset_variables()
                events_on_file = os.path.join(events_dir,'taps_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '_events_on')
                events_off_file = os.path.join(events_dir,'taps_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) +  '_events_off')
                events_video = os.path.join(video_dir,'event_stream_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '.mp4')
                acc_video = os.path.join(video_dir,'accumulator_trial_' +str(trial_idx) +'_pose_'  + str(pose_idx) + '.mp4')                    
                sensor.set_filenames(events_on_file = events_on_file, events_off_file = events_off_file, events_video_file = events_video, acc_video_file = acc_video)

                robot.move_linear(obj_poses[pose_idx])
                
                # Contact perspex
                robot.speed = 4
              

                # Start sensors                
                sensor.start_logging()    
                t_in = threading.Thread(target=sensor.get_events, args = ())
                t_in.start()
                
                # Tap
                print("Trial " + str(trial_idx+1) + "/" + str(n_trials) + " Pose " + str((pose_idx+1)) + "/" + str(len(obj_poses)))  
                
                tap_1 = [sum(x) for x in zip(tap_move[0], obj_poses[pose_idx])]
                tap_2 = [sum(x) for x in zip(tap_move[1], obj_poses[pose_idx])]
                robot.move_linear(tap_1)
                time.sleep(0.1)
                robot.move_linear(tap_2)

                
                # Stop recording
                sensor.stop_logging()
                t_in.join()
                
                # Save data        

                sensor.value_cleanup()
                # sensor.save_events_stream()
                
                sensor.save_events_on()
                sensor.save_events_off()     


        # Return home
        print("Moving home ...")
        robot.speed = linear_speed
        robot.move_linear(home_pose)  # Move home
        
def main():

    # Directories
    collect_dir_name = os.path.join('horizontal_shear', 'horizontal_shear_' + time.strftime('%m%d%H%M'))
    collect_dir = os.path.join(os.environ['DATAPATH'], sensor_type, collect_dir_name)
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

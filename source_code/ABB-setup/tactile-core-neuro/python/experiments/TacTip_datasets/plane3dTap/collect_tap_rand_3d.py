# -*- coding: utf-8 -*-
"""
Created on Fri Feb  1 09:58:05 2019

@author: nl13426
"""

import os, time, json, shutil
import numpy as np
import pandas as pd

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor

def make_robot():
    return AsyncRobot(SyncRobot(ABBController(ip='164.11.72.43')))

def make_sensor():
    camera=CvVideoCamera(source=0, exposure=-6)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvVideoOutputFile(),
        ))

def main():
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip-datasets', 'edge5dTap')
    collect_dir_name = 'collect_tap_rand_2d_' + time.strftime('%m%d%H%M')

    # experiment metadata
    collect_dir = os.path.join(home_dir, collect_dir_name)
    video_dir = os.path.join(collect_dir, 'videos')
    video_target_file = os.path.join(collect_dir, 'targets_video.csv')
    robot_tcp = [0, 0, 89.1, 0, 0, 0]
    base_frame = [0, 0, 0, 0, 0, 0]
    home_pose = [400, 0, 300, 180, 0, 180]
    work_frame = [138, 392, 93, 180, 0, 180]
    linear_speed = 20
    angular_speed = 10
    num_frames = 1
    tap_move = [[0, 0, 5, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
    poses_rng = [[0, 0, -4, -15, -15, 0], [ 0, 0, 0, 15, 15, 0]]
    obj_poses = [[ 0, 0, 0, 0, 0, 0]]
    num_poses = 5

    # create dir
    os.makedirs(collect_dir)
    
    # save metadata
    meta = locals().copy()
    with open(os.path.join(collect_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f)

    # generate random poses        
    np.random.seed()
    poses = np.random.uniform(low=poses_rng[0], high=poses_rng[1], size=(num_poses, 6))
    poses = poses[np.lexsort((poses[:,0], poses[:,5]))]
                 
    # generate and save target data
    target_df = pd.DataFrame(columns=['sensor_video', 'obj_id', 'pose_id',
        'pose_1', 'pose_2', 'pose_3', 'pose_4', 'pose_5', 'pose_6'])
    for i in range(num_poses * len(obj_poses)):
        video_file = 'video_{:d}.mp4'.format(i + 1)
        i_pose, i_obj = (int(i % num_poses), int(i / num_poses))
        pose = poses[i_pose, :] + obj_poses[i_obj]
        target_df.loc[i] = np.hstack((video_file, i_obj+1, i_pose+1, pose))
    target_df.to_csv(video_target_file, index=False)

    # Collect videos in temp directory
    extract_video_dir = os.path.join(os.environ['TEMPPATH'], collect_dir_name+'Videos')
    os.makedirs(extract_video_dir)

    # collect data                
    with make_robot() as robot, make_sensor() as sensor:
        # configure robot
        robot.tcp = robot_tcp
        
        # grab initial frames from sensor
        sensor.process(num_frames=1+num_frames, outfile=os.path.join(extract_video_dir, 'init.mp4'))    

        # move to home 
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)
        
        # move to work frame origin; set work speed
        robot.coord_frame = work_frame
        robot.move_linear([0, 0, 0, 0, 0, 0])        
        robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)

        # iterate over objects and poses
        for index, row in target_df.iterrows():
            i_pose, i_obj = (int(row.loc['obj_id']), int(row.loc['pose_id']))
            pose = row.loc['pose_1' : 'pose_6'].values
            sensor_video = row.loc['sensor_video']
            print(f"Collecting data for object {i_obj}, pose {i_pose} ...")
            
            # move to new pose
            robot.move_linear(pose)
            
            # make tap move and capture data
            robot.coord_frame = base_frame
            robot.coord_frame = robot.pose
            robot.move_linear(tap_move[0])
            sensor.process(num_frames=1+num_frames, outfile=os.path.join(extract_video_dir, sensor_video))
            robot.move_linear(tap_move[1])
            robot.coord_frame = work_frame

        # move to home position
        print("Moving to home position ...")
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)

    # archive video dir
    shutil.make_archive(video_dir, 'zip', extract_video_dir)
    shutil.rmtree(extract_video_dir)
           
if __name__ == '__main__':
    main()

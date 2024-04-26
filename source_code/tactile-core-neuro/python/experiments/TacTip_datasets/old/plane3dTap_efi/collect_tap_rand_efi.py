# -*- coding: utf-8 -*-
    
import os, time, json, shutil
import numpy as np
import pandas as pd

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot():
    return AsyncRobot(SyncRobot(ABBController(ip='164.11.72.111', port=5000))) 
#    return AsyncRobot(SyncRobot(ABBController(ip='127.0.0.1', port=5000)))  

def make_sensor():
    camera=CvVideoCamera(source=0, exposure=-6)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvVideoOutputFile(),
        ))
    
def make_meta(meta_file,              
              robot_tcp = [-6.5, 0, 78, 0, 0, -45],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [400, 0, 300, 180, 0, 180],
              work_frame = [215, 390, 99.5+5, 180, 0, 0],
              linear_speed = 20,
              angular_speed = 10,
              num_frames = 1,
              tap_move = [[0, 0, 5, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
              poses_rng = [[0, 0, -2, -8, -8, 0], [0, 0, 0, 8, 8, 0]],
              obj_poses = [[0, 0, 0, 0, 0, 0]],
              num_poses = 500, 
              ):
    data_dir = os.path.dirname(meta_file)

    video_dir = os.path.join(data_dir, 'videos')
    video_df_file = os.path.join(data_dir, 'targets_video.csv')

    meta = locals().copy()
    del meta['data_dir']
    return meta
    
def make_video_df(poses_rng, num_poses, obj_poses, video_df_file, **kwargs):   
    # generate random poses
    np.random.seed()
    poses = np.random.uniform(low=poses_rng[0], high=poses_rng[1], size=(num_poses, 6))
    poses = poses[np.lexsort((poses[:,2], poses[:,5]))]
                 
    # generate and save target data
    video_df = pd.DataFrame(columns=['sensor_video', 'obj_id', 'pose_id',
        'pose_1', 'pose_2', 'pose_3', 'pose_4', 'pose_5', 'pose_6'])
    for i in range(num_poses * len(obj_poses)):
        video_file = 'video_{:d}.mp4'.format(i + 1)
        i_pose, i_obj = (int(i % num_poses), int(i / num_poses))
        pose = poses[i_pose, :] + obj_poses[i_obj]
        video_df.loc[i] = np.hstack((video_file, i_obj+1, i_pose+1, pose))
    video_df.to_csv(video_df_file, index=False)
    
def collect_tap(video_dir, video_df_file, num_frames, tap_move, robot_tcp, 
                base_frame, home_pose, work_frame, linear_speed, angular_speed, **kwargs):    
    os.makedirs(video_dir)
    video_df = pd.read_csv(video_df_file)
    
    with make_robot() as robot, make_sensor() as sensor:       
        # grab initial frames from sensor
        sensor.process(num_frames=1+num_frames, outfile=os.path.join(video_dir, 'video_init.mp4'))    

        # initialize robot to home         
        robot.tcp = robot_tcp
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)
        robot.move_joints(np.append(robot.joint_angles[:-1], robot_tcp[-1]))
        
        # move to work frame origin; set work speed
        robot.coord_frame = work_frame
#        robot.move_linear([0, 0, -100, 0, 0, 0])        
        robot.move_linear([0, 0, 0, 0, 0, 0])        
        robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)

        # iterate over objects and poses
        for index, row in video_df.iterrows():
            i_obj, i_pose = (int(row.loc['obj_id']), int(row.loc['pose_id']))
            pose = row.loc['pose_1' : 'pose_6'].values
            sensor_video = row.loc['sensor_video']
            print(f"Collecting data for object {i_obj}, pose {i_pose} ...")
            
            # move to new pose
            robot.move_linear(pose)
            
            # make tap move and capture data
            robot.coord_frame = base_frame
            robot.coord_frame = robot.pose
            robot.move_linear(tap_move[0])
            sensor.process(num_frames=1+num_frames, 
                           outfile=os.path.join(video_dir, sensor_video))
            robot.move_linear(tap_move[1])
            robot.coord_frame = work_frame

        # move to home position
        print("Moving to home position ...")
        robot.coord_frame = base_frame
        robot.linear_speed, robot.angular_speed = (50, 50)
        robot.move_linear(home_pose)
        robot.move_joints(np.append(robot.joint_angles[:-1], 0))


def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\plane3dTap')
    meta_file = os.path.join(home_dir, 
         os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')
    
    # Make and save metadata and pose dataframe
    meta = make_meta(meta_file)    
    os.makedirs(os.path.dirname(meta_file))
    make_video_df(**meta)
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)   
    
    # Save images to temporary folder
    temp_meta = {**meta, 'video_dir': os.environ['TEMPPATH'] + r'\videos'+time.strftime('%m%d%H%M')}
    
    # Collect data
    collect_tap(**temp_meta)
    
    # Tidy up
    shutil.make_archive(meta['video_dir'], 'zip', temp_meta['video_dir'])
    shutil.rmtree(temp_meta['video_dir'])
               
if __name__ == '__main__':
    main()

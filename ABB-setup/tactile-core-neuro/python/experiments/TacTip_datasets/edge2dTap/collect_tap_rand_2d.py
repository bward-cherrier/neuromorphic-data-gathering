# -*- coding: utf-8 -*-
    
import os, time, json, shutil
import numpy as np
import pandas as pd

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from core.utils.preprocVideoCamera import CvPreprocVideoCamera
from vsp.video_stream import CvImageOutputFileSeq, CvVideoDisplay #, CvimageOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot(ip):
    return AsyncRobot(SyncRobot(ABBController(ip=ip, port=5000)))

def make_sensor(size=None, crop=None, threshold=None, exposure=-6):
    camera=CvPreprocVideoCamera(size, crop, threshold, source=0, exposure=exposure)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))
    
def make_meta(meta_file,   
              ip = '164.11.72.16',
              robot_tcp = [0, 0, 89.1, 0, 0, 0],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [400, 0, 300, 180, 0, 180],
              work_frame = [158, 392, 93, 180, 0, 180],
              linear_speed = 20,
              angular_speed = 10,
              num_frames = 1,
              exposure = -6,
              size = None,
              crop = [160, 70, 490, 400],
              threshold = (11, -5), 
              tap_move = [[0, 0, 5, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
              poses_rng = [[-5, 0, -1, 0, 0, -45], [5, 0, 1, 0, 0, 45]],
              obj_poses = [[0, 0, 0, 0, 0, 0]],
              num_poses = 50, 
              ):
    data_dir = os.path.dirname(meta_file)

    image_dir = os.path.join(data_dir, 'images_bw')
    image_df_file = os.path.join(data_dir, 'targets_image.csv')

    meta = locals().copy()
    del meta['data_dir']
    return meta
    
def make_image_df(poses_rng, num_poses, obj_poses, image_df_file, **kwargs):   
    # generate random poses
    np.random.seed()
    poses = np.random.uniform(low=poses_rng[0], high=poses_rng[1], size=(num_poses, 6))
    poses = poses[np.lexsort((poses[:,2], poses[:,5]))]
                 
    # generate and save target data
    image_df = pd.DataFrame(columns=['sensor_image', 'obj_id', 'pose_id',
        'pose_1', 'pose_2', 'pose_3', 'pose_4', 'pose_5', 'pose_6'])
    for i in range(num_poses * len(obj_poses)):
        image_file = 'image_{:d}.png'.format(i + 1)
        i_pose, i_obj = (int(i % num_poses), int(i / num_poses))
        pose = poses[i_pose, :] + obj_poses[i_obj]
        image_df.loc[i] = np.hstack((image_file, i_obj+1, i_pose+1, pose))
    image_df.to_csv(image_df_file, index=False)
        
def collect_tap(image_dir, image_df_file, num_frames, tap_move, robot_tcp, 
                ip, size, crop, threshold, exposure,
                base_frame, home_pose, work_frame, linear_speed, angular_speed, **kwargs):    
    os.makedirs(image_dir)
    image_df = pd.read_csv(image_df_file)
    
    with make_robot(ip) as robot, make_sensor(size, crop, threshold, exposure) as sensor:       
        # grab initial frames from sensor
        sensor.process(num_frames=1+num_frames, outfile=os.path.join(image_dir, 'image_init.png'))      

        # initialize robot to home         
        robot.tcp = robot_tcp
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)
        
        # move to work frame origin; set work speed
        robot.coord_frame = work_frame
        robot.move_linear([0, 0, 0, 0, 0, 0])        
        robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)

        # iterate over objects and poses
        for index, row in image_df.iterrows():
            i_obj, i_pose = (int(row.loc['obj_id']), int(row.loc['pose_id']))
            pose = row.loc['pose_1' : 'pose_6'].values.astype(np.float)            
            sensor_image = row.loc['sensor_image'].replace('_0','')            
            with np.printoptions(precision=2, suppress=True):
                print(f'Collecting data for object {i_obj}, pose {i_pose}: ...')
                print(f'pose = {pose}'.replace('. ',''))
                print(f'joints = {robot.joint_angles}'.replace('. ',''))
    
            # move to new pose
            robot.move_linear(pose)
            
            # make tap move and capture data
            robot.coord_frame = base_frame
            robot.coord_frame = robot.target_pose
            robot.move_linear(tap_move[0])
            sensor.process(num_frames=1+num_frames, 
                           outfile=os.path.join(image_dir, sensor_image))
            robot.move_linear(tap_move[1])
            robot.coord_frame = work_frame

        print("Moving to home position ...")
        robot.move_linear([0, 0, -100, 0, 0, 0])        
        robot.coord_frame = base_frame
        robot.linear_speed, robot.angular_speed = (50, 50)
        robot.move_linear(home_pose)
        robot.move_joints(np.append(robot.joint_angles[:-1], 0))

def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'edge2dTap')
    meta_file = os.path.join(home_dir, 
         os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')
    
    # Make and save metadata and pose dataframe
    meta = make_meta(meta_file)    
    os.makedirs(os.path.dirname(meta_file))
    make_image_df(**meta)
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)   
    
    # Save images to temporary folder
    temp_meta = {**meta, 'image_dir': os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))}
    
    # Collect data
    collect_tap(**temp_meta)
    
    # Tidy up
    shutil.make_archive(meta['image_dir'], 'zip', temp_meta['image_dir'])
    shutil.rmtree(temp_meta['image_dir'])
               
if __name__ == '__main__':
    main()

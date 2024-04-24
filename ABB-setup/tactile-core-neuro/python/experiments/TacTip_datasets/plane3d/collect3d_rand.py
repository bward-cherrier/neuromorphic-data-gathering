# -*- coding: utf-8 -*-
    
import os, time, json, shutil
import numpy as np
import pandas as pd

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from core.utils.preproc_video_camera import CvPreprocVideoCamera
from vsp.video_stream import CvImageOutputFileSeq, CvVideoDisplay #, CvimageOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot(ip):
    return AsyncRobot(SyncRobot(ABBController(ip=ip, port=5000)))

# NB amcap: reset all settings; autoexposure off; saturdation max; beware MS camera app
def make_sensor(size=None, crop=None, threshold=None, exposure=-6):
    camera=CvPreprocVideoCamera(size, crop, threshold, source=0, exposure=exposure)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))
  
# top of ball: 396, -135, 239.3, 180, 0, 0
# middle of disk: 210, 395, 93.5, 180, 0, 0
# edge of disk: 155, 392, 92.8, 180, 0, 0    
    
def make_meta(home_dir, meta_file,
              ip = '164.11.72.57',
              robot_tcp = [0, 0, 89.1, 0, 0, -180],
              base_frame = [0, 0, 0, 0, 0, 0],
              home_pose = [400, 0, 300, 180, 0, 180],
              work_frame = [210, 395, 93.5, 180, 0, 180],
              linear_speed = 20,
              angular_speed = 10,
              num_frames = 1,
              exposure = -6,
              size = None,
              crop = [160, 70, 490, 400],
              threshold = (11, -5), 
              poses_rng = [[0, 0, 1, -15, -15, 0], [0, 0, 5, 15, 15, 0]],
              moves_rng = [[-5, -5, 0, -5, -5, -5], [5, 5, 0, 5, 5, 5]], 
              obj_poses = [[0, 0, 0, 0, 0, 0]],
              num_poses = 2000, 
              ):
    data_dir = os.path.dirname(meta_file)
    
    image_dir = os.path.join(data_dir, 'images_bw')
    image_df_file = os.path.join(data_dir, 'targets_image.csv')

    meta = locals().copy()
    del meta['data_dir']
    return meta
    
def make_image_df(poses_rng, moves_rng, num_poses, obj_poses, image_df_file, **kwargs):
    # generate random poses
    np.random.seed()
    poses = np.random.uniform(low=poses_rng[0], high=poses_rng[1], size=(num_poses, 6))
    poses = poses[np.lexsort((poses[:,1], poses[:,5]))]
    moves = np.random.uniform(low=moves_rng[0], high=moves_rng[1], size=(num_poses, 6))
              
    # generate and save target data
    image_df = pd.DataFrame(columns=['sensor_image', 'obj_id', 'pose_id',
        'pose_1', 'pose_2', 'pose_3', 'pose_4', 'pose_5', 'pose_6',
        'move_1', 'move_2', 'move_3', 'move_4', 'move_5', 'move_6'])
    for i in range(num_poses * len(obj_poses)):
        video_file = 'image_{:d}_0.png'.format(i + 1)
        i_pose, i_obj = (int(i % num_poses), int(i / num_poses))
        pose = poses[i_pose, :] + obj_poses[i_obj]
        move = moves[i_pose, :]
        image_df.loc[i] = np.hstack((video_file, i_obj+1, i_pose+1, pose, move))
    return image_df

def collect(image_dir, image_df_file, num_frames, robot_tcp, 
            ip, size, crop, threshold, exposure,
            base_frame, home_pose, work_frame, linear_speed, angular_speed, **kwargs):    
    os.makedirs(image_dir)
    image_df = pd.read_csv(image_df_file)
    
    with make_robot(ip) as robot, make_sensor(size, crop, threshold, exposure) as sensor:       
        # grab initial frames from sensor
        sensor.process(num_frames=1+num_frames, 
                outfile=os.path.join(image_dir, 'image_init.png'))      

        # move to home position
        robot.tcp = robot_tcp
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)
        
        # move to start position; set work speed
        robot.coord_frame = work_frame
        robot.move_linear([0, 0, 0, 0, 0, 0])        
        robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)

        # iterate over objects and poses
        for index, row in image_df.iterrows():
            i_obj, i_pose = (int(row.loc['obj_id']), int(row.loc['pose_id']))
            pose = row.loc['pose_1' : 'pose_6'].values.astype(np.float)
            move = row.loc['move_1' : 'move_6'].values.astype(np.float)
            sensor_image = row.loc['sensor_image'].replace('_0','')            
            with np.printoptions(precision=2, suppress=True):
                print(f'Collecting data for object {i_obj}, pose {i_pose}: ...')
                print(f'pose = {pose}'.replace('. ',''))
                print(f'joints = {robot.joint_angles}'.replace('. ',''))            
            
            # move to new pose (avoid changing pose in contact with object)
            robot.move_linear(pose - move + [0, 0, -5, 0, 0, 0])
            robot.move_linear(pose - move)
            robot.move_linear(pose)
            sensor.process(num_frames=1+num_frames, 
                           outfile=os.path.join(image_dir, sensor_image))
            robot.move_linear(pose + [0, 0, -5, 0, 0, 0])

        print("Moving to home position ...")
        robot.move_linear([0, 0, -100, 0, 0, 0])        
        robot.coord_frame = base_frame
        robot.linear_speed, robot.angular_speed = (50, 50)
        robot.move_linear(home_pose)
        robot.move_joints(np.append(robot.joint_angles[:-1], 0))
      
#def main():    
# Specify directories and files
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
meta_file = os.path.join('edge3d', os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 
                         'meta.json')

# Make and save meta data and dataframe
meta = make_meta(home_dir, meta_file) 
image_df = make_image_df(**meta)

os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))
with open(os.path.join(home_dir, meta_file), 'w') as f: 
    json.dump(meta, f)   
image_df.to_csv(os.path.join(home_dir, meta['image_df_file']), index=False)

# Temporary folder
temp_dir = os.path.join(os.environ['TEMPPATH'], 'images' + time.strftime('%m%d%H%M'))
image_dir = os.path.join(home_dir, meta['image_dir'])
meta['image_dir'] = os.path.join(temp_dir, meta['image_dir'])
meta['image_df_file'] = os.path.join(home_dir, meta['image_df_file'])

# Collect data
collect(**meta)

# Zip data
shutil.make_archive(image_dir, 'zip', meta['image_dir'])
shutil.rmtree(meta['image_dir'])
               
#if __name__ == '__main__':
#    main()

# -*- coding: utf-8  v  -*-
  
import os, json, warnings, math, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
import scipy.io

from copy import copy
from core.model.cnn.regression_cnn import RegressionCNN
from core.utils.plotContour import PlotContour
from core.utils.plotFrames import PlotFrames
from core.utils.plotControl import PlotControl

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController
from cri.robot import quat2euler, euler2quat, inv_transform

def make_robot(ip='127.0.0.1'):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 

def control_cnn_replay(robot, model, test_image_dir, target_names,
                robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed,
                crop, size, threshold, 
                num_steps = 10,
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-math.inf,]*6, [math.inf,]*6],
                target_poses = np.array([[0,]*6, [0,]*6]),
                target_rng = [[-np.inf,]*6, [np.inf,]*6],
                **kwargs):
    y = np.zeros((num_steps+1, 6))
    e = np.zeros((num_steps+1, 6))
    ei = np.zeros((num_steps+1, 6))
    u = np.zeros((num_steps+1, 6))
    v = np.zeros((num_steps+1, 6))
    ei[0,] = [0,]*6
    u[0,] = copy(r)
    v[0,] = copy(r)
    y[0,] = copy(r)

    pose = ['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']
    target_inds = [pose.index(t) for t in target_names]
  
    # initialize robot to home         
    robot.tcp = robot_tcp
    robot.coord_frame = base_frame
    robot.linear_speed = 50
    robot.move_linear(home_pose)
    robot.move_joints(np.append(robot.joint_angles[:-1], robot_tcp[-1]))
    
    # move to work frame origin; set work speed
    robot.coord_frame = work_frame
    robot.move_linear([0, 0, -100, 0, 0, 0])        
    robot.move_linear([0, 0, 0, 0, 0, 0])        
    robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)

    # open plot
    fig = [PlotContour(target_poses), PlotFrames()]#, PlotControl(target_inds)]

    # step method
    pred_df = pd.DataFrame()
    for i in range(num_steps):
        test_file = os.path.join(test_image_dir, f'image{i+1}.mat')
            
        # control signal in sensor frame
        e[i,] = r - y[i,]
        ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,], ei_bnd[0]), ei_bnd[1])
        u[i+1,] = kp*e[i,] + ki*ei[i+1,]
        
#            # 3D CASE: correct for transformation on uncontrolled variables
#            if len(target_names)==3 and r[-1]==0:
#                u[i+1,-1] = u[i+1,-1] - mat.transFrame(u[i+1,], v[i,])[:,-1]

        # transform control signal to sensor frame                
        u_q = euler2quat(u[i+1,], axes='rxyz')
        v_q = euler2quat(v[i,], axes='rxyz')
        v[i+1,] = quat2euler(inv_transform(u_q, v_q), axes='rxyz')

        # joint limits
        if np.any(robot.joint_angles[5]>245+20): robot.move_joints(robot.joint_angles - [0,0,0,0,0,360])
        if np.any(robot.joint_angles[5]<-360): robot.move_joints(robot.joint_angles + [0,0,0,0,0,360])
                    
        # move to new pose
        robot.move_linear(v[i+1,])
            
        # capture data
        image = np.array(scipy.io.loadmat(test_file)['image'])
        
        # make prediction
        y[i+1,target_inds] = model.predict(image, crop=crop, size=tuple(size), threshold=threshold)    
        with np.printoptions(precision=2, suppress=True):
            print(f'{i+1}: e={e[i,]}'.replace('. ',''))
            print(f'joints={robot.joint_angles}'.replace('. ',''))
    
        pred_row = pd.Series({'sensor_image': os.path.basename(test_file)})
        for j, jtem in enumerate(pose):
            pred_row['pose_'+str(j+1)] = v[i+1,j]
            pred_row['pred_'+str(j+1)] = y[i+1,j]
        pred_df = pred_df.append(pred_row, ignore_index=True)
        
        # update plot
        fig[0].update(v[:i+1,])
#        fig[1].update(image[40:440,120:520,]/255, y[i+1,])
#        fig[2].update(i, y, e, ei)

    # move to home position
    print("Moving to home position ...")
    robot.move_linear([0, 0, -100, 0, 0, 0])        
    robot.coord_frame = base_frame
    robot.linear_speed, robot.angular_speed = (50, 50)
    robot.move_linear(home_pose)
    robot.move_joints(np.append(robot.joint_angles[:-1], 0))
        
    return pred_df, fig


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol', 'explore5d')
    replay_dir = os.path.join(home_dir, 'explore5dCNN08091412')

    with open(os.path.join(replay_dir, 'meta.json'), 'r') as f:
        meta = json.load(f)
    meta['work_frame'] = np.array(meta['work_frame']) - [0, 0, 20, 0, 0, 0] 
    meta['linear_speed'], meta['angular_speed'] = (100, 100)

    # unpack to temporary folder
    meta['test_image_dir'] = os.path.join(os.environ['TEMPPATH'], 'data')
    shutil.unpack_archive(meta['data_image_dir']+r'.zip', meta['test_image_dir'])

    # target trajectory
    poses_path = os.path.join(replay_dir, 'predictions.csv')
    meta['target_poses'] = pd.read_csv(poses_path).loc[:,'pose_1':'pose_6'].values

    # startup/load model and make predictions on test data
    model = RegressionCNN()
    model.load_model(**meta)

    # Collect data
    with make_robot() as robot:
        pred_df, fig = control_cnn_replay(robot, model, **meta)
#    pred_df.to_csv(meta['test_df_file'])
#    fig[0].finish(os.path.join(home_dir, os.path.dirname(meta_file), 'contour.png'))         
#    fig[1].finish(os.path.join(home_dir, replay_dir, file, 'frames'))         
#    fig[2].finish(os.path.join(home_dir, replay_dir, file, 'control'))         

    # Tidy up
    shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

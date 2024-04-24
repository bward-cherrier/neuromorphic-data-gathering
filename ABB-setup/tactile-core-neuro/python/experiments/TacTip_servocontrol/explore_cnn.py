# -*- coding: utf-8  v  -*-
  
import os, json, warnings, cv2, math, time, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from win32api import GetSystemMetrics
from copy import copy
from core.model.cnn.regression_cnn import RegressionCNN
import transforms3d as t3d
from transforms3d.quaternions import qinverse, qmult, rotate_vector
from cri.robot import quat2euler, euler2quat, inv_transform, transform
import scipy.io

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot():
    return AsyncRobot(SyncRobot(ABBController(ip='164.11.72.92')))

def make_sensor():
    camera=CvVideoCamera(source=0, exposure=-6)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvVideoOutputFile(),
        ))
    
def control_cnn_replay(model, data_image_dir, target_names,
                robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed,
                crop, size, threshold, 
                num_steps = 10,
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-math.inf,]*6,[math.inf,]*6],
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
  
    with make_robot() as robot:
        # initialize robot to home         
        robot.tcp = robot_tcp
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)
        
        # move to work frame origin; set work speed
        robot.coord_frame = work_frame
        robot.move_linear([0, 0, 0, 0, 0, 0])        
        robot.linear_speed, robot.angular_speed = (linear_speed, angular_speed)
       
        h = PlotContour()
    
        # step method
        pred_df = pd.DataFrame()
        for i in range(num_steps):
            test_mat_file = os.path.join(data_image_dir, f'image{i+1}.mat')
                
            # control signal in sensor frame
            e[i,] = r - y[i,]
            ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,],ei_bnd[0]), ei_bnd[1])
            u[i+1,] = kp*e[i,] + ki*ei[i+1,]
            v[i+1,] = trans_frame(u[i+1,:], v[i,:])
            
            # move to new pose
            robot.move_linear(v[i+1,])
            
            # capture data
#            image = sensor.process(num_frames=1+num_frames)
            
            # make prediction
            y[i+1,target_inds] = model.predict(image, crop=crop, size=tuple(size), threshold=threshold)    
            with np.printoptions(precision=2, suppress=True):
                print(f'{i+1}: y={y[i+1,]}, e={e[i,]}, v={v[i+1,]}'.replace('. ',''))
        
            pred_row = pd.Series({'sensor_image': os.path.basename(test_mat_file)})
            for j, jtem in enumerate(pose):
                pred_row['pose_'+str(j+1)] = v[i+1,j]
                pred_row['pred_'+str(j+1)] = y[i+1,j]
            pred_df = pred_df.append(pred_row, ignore_index=True)
            
            h.update(v[i+1,])

        # move to home position
        print("Moving to home position ...")
        robot.coord_frame = base_frame
        robot.linear_speed = 50
        robot.move_linear(home_pose)

    h.finish()
    return pred_df    

def trans_frame(pose_e, frame_e):
    pose_q = euler2quat(pose_e, axes='sxyz')
    frame_q = euler2quat(frame_e, axes='sxyz')
    pos_q = frame_q[:3] + rotate_vector(pose_q[:3], qinverse(frame_q[3:]))
    rot_q = qmult(frame_q[3:], pose_q[3:])
    return quat2euler(np.concatenate((pos_q, rot_q)), axes='sxyz')

class PlotContour:
    def __init__(self):
        plt.ion()
        self._fig = plt.figure(figsize=(3.5,3.5), num=2)
        self._ax = self._fig.add_subplot(111, projection='3d')
        self._ax.plot([], [], [], '-r')  
           
        geom = self._fig.canvas.manager.window.geometry()      
        x,y,dx,dy = geom.getRect()
        sx,sy = GetSystemMetrics(0), GetSystemMetrics(1)
        self._fig.canvas.manager.window.setGeometry(sx-dx,sy-dy,dx,dy)
        plt.draw()
        
    def update(self, u):
        xdata, ydata, zdata = self._ax.lines[0]._verts3d
        xd = np.append(xdata, u[0])
        yd = np.append(ydata, u[1])
        zd = np.append(zdata, u[2])
        self._ax.set(xlim=[np.min(xd)-0.5,np.max(xd)+0.5], ylim=[np.min(yd)-0.5,np.max(yd)+0.5])
        self._ax.lines[0].set(xdata=xd, ydata=yd)
        self._ax.set(zlim=[np.min(zd)-0.5,np.max(zd)+0.5])
        self._ax.lines[0].set_3d_properties(zd)
        plt.draw()
        plt.pause(0.001)

    def finish(self):
        plt.show(block=True)


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol\\explore2d')
    replay_meta_file = os.path.join(home_dir, 'explore2dCNN08081209\\meta.json1')

    with open(replay_meta_file, 'r') as f:
        meta = json.load(f)

    meta['data_image_dir'] = os.path.join(home_dir, 'explore2dCNN08081209\\imagesPython')
    meta['model_file'] = os.path.join(home_dir, 'train2dCNN07082051\\best_model.h5')
    meta['work_frame'] = [158, 392, 89, 180, 0, 180]
    meta['num_steps'] = 100

    # unpack to temporary folder
    temp_meta = {**meta, 'data_image_dir': os.environ['TEMPPATH'] + r'\data'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['data_image_dir']+r'.zip', temp_meta['data_image_dir'])

    # startup/load model and make predictions on test data
    cnn = RegressionCNN()
    cnn.load_model(**temp_meta)
    cnn.print_model_summary()

    # Collect data
    pred_df = control_cnn_replay(cnn, **temp_meta)
    pred_df.to_csv(os.path.dirname(replay_meta_file) +r'\predictions_python.csv')

    # Tidy up
    shutil.rmtree(temp_meta['data_image_dir'])

if __name__ == '__main__':
    main()

#
#from cri.robot import SyncRobot, AsyncRobot
#from cri.controller import ABBController
#
#def make_robot():
#    return AsyncRobot(SyncRobot(ABBController(ip='164.11.72.92')))
#
#base_frame = [0, 0, 0, 0, 0, 0]
#home_pose = [400, 0, 300, 180, 0, 180]
#with make_robot() as robot:
#    robot.coord_frame = base_frame
#    robot.linear_speed = 50
#    robot.move_linear(home_pose)
# -*- coding: utf-8  v  -*-
  
import os, json, warnings, math, time, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from copy import copy
from core.model.dnn.CNN_model import CNNmodel
from core.utils.control_matlab import ControlMatlab
from explore_cnn_replay import PlotContour # trans_frame
from experiments.TacTip_datasets.extract_images import process_image

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvImageOutputFileSeq
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot(ip):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 

def make_sensor():
    camera=CvVideoCamera(source=0, exposure=-6)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))

def make_test_meta(meta_file, model_meta_file,  
                   ip = '164.11.72.16',                                          
                   r = [0, 1, 0, 0, 0, 0],
                   kp = [0.5, 1, 0, 0, 0, 0.5],
                   ki = [0.3, 1, 0, 0, 0, 0.1],
                   ei_bnd = [[-5, 0, 0, 0, 0, -45], [5, 0, 0, 0, 0, 45]],
                   num_steps = 240,
                   ):
    test_dir =  os.path.dirname(meta_file)
    model_dir =  os.path.dirname(model_meta_file)
        
    # load metadata
    with open(model_meta_file, 'r') as f: 
        meta = json.load(f)

    # update metadata
    test_video_dir = os.path.join(test_dir, 'videos')
    test_df_file = os.path.join(test_dir, 'predictions.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))

    # ensure appropriate type
    size = tuple(meta['size']) 
    
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta

        
def control_dnn(model, test_video_dir, target_names,
                ip, robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed,
                num_frames, tap_move, crop, size, threshold, 
                num_steps = 10,
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-math.inf,]*6, [math.inf,]*6],
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

    os.makedirs(test_video_dir)
    pose = ['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']
    target_inds = [pose.index(t) for t in target_names]
  
    with make_robot(ip) as robot, make_sensor() as sensor:       
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
       
        h = PlotContour()
        mat = ControlMatlab()
    
        # step method
        pred_df = pd.DataFrame()
        for i in range(num_steps):
            test_file = os.path.join(test_video_dir, f'image_{i+1}.png')
                
            # control signal in sensor frame
            e[i,] = r - y[i,]
            ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,], ei_bnd[0]), ei_bnd[1])
            u[i+1,] = kp*e[i,] + ki*ei[i+1,]
            
            # 3D CASE: correct for transformation on uncontrolled variables
            if len(target_names)==3 and r[-1]==0:
                u[i+1,-1] = u[i+1,-1] - mat.transFrame(u[i+1,], v[i,])[:,-1]
            
#            v[i+1,] = trans_frame(u[i+1,], v[i,])
            v[i+1,] = mat.transFrame(u[i+1,], v[i,])
            
            # move to new pose
            robot.move_linear(v[i+1,])
                
            # move to tap location and tke data
            robot.coord_frame = base_frame
            robot.coord_frame = robot.target_pose
            robot.move_linear(tap_move[0])
            image = sensor.process(num_frames=1+num_frames, outfile=test_file)
            robot.move_linear(tap_move[1])
            robot.coord_frame = work_frame

            image = process_image(image[1,:,:,:], crop, size, threshold)
            image = image[np.newaxis, ...].astype('float32') / 255     
        
            # make prediction
            y[i+1,target_inds] = model.predict(image)    
            with np.printoptions(precision=2, suppress=True):
                print(f'{i+1}: y={y[i+1,]}, e={e[i,]}, v={v[i+1,]}'.replace('. ',''))
                print(f'joints={robot.joint_angles}'.replace('. ',''))
        
            pred_row = pd.Series({'sensor_image': os.path.basename(test_file)})
            for j, jtem in enumerate(pose):
                pred_row['pose_'+str(j+1)] = v[i+1,j]
                pred_row['pred_'+str(j+1)] = y[i+1,j]
            pred_df = pred_df.append(pred_row, ignore_index=True)
            
            h.update(v[i+1,])

        # move to home position
        print("Moving to home position ...")
        robot.move_linear([0, 0, -100, 0, 0, 0])        
        robot.coord_frame = base_frame
        robot.linear_speed, robot.angular_speed = (50, 50)
        robot.move_linear(home_pose)
        robot.move_joints(np.append(robot.joint_angles[:-1], 0))
        
    h.finish() 
    return pred_df     


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'edge2dtap')
    model_meta_file = os.path.join(home_dir, 'collect_tap_rand_2d_02072153', 'models', 'train_dnn_2d_02072226', 'meta.json')
    
    with open(model_meta_file, 'r') as f:
        meta = json.load(f)
    
    # Make test meta
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol', 'explore2d', test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, model_meta_file)
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
            json.dump(meta, f)
    
    # temporary image folder
    temp_meta = {**meta, 'test_video_dir': os.path.join(os.environ['TEMPPATH'], 'videos'+time.strftime('%m%d%H%M'))}
    
    # startup/load model and make predictions on test data
    cnn = CNNmodel()
    cnn.load_model(**meta)
    cnn.print_model_summary()
    
    # Collect data
    pred_df = control_dnn(cnn, **temp_meta)
    pred_df.to_csv(temp_meta['test_df_file'])
    
    # Tidy up
    shutil.make_archive(meta['test_video_dir'], 'zip', temp_meta['test_video_dir'])
    shutil.rmtree(temp_meta['test_video_dir'])

if __name__ == '__main__':
    main()

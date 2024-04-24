# -*- coding: utf-8 
  
import os, json, warnings, time, shutil, cv2
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np

from copy import copy
from core.model.dnn.CNN_model import CNNmodel
from core.utils.plotContour import PlotContour
from core.utils.plotFrames import PlotFrames
from core.utils.plotControl import PlotControl

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController
from cri.robot import quat2euler, euler2quat, inv_transform

from core.utils.preproc_video_camera import CvPreprocVideoCamera
from vsp.video_stream import CvImageOutputFileSeq, CvVideoDisplay #, CvimageOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot(ip='164.11.72.57'):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 

# NB set autoexposure off in amcap; reset all settings amcap; don't use ms camera app; move saturdation to right in amcap (greyscale)
def make_sensor(size=None, crop=None, threshold=None, exposure=-6, **kwargs):
    camera=CvPreprocVideoCamera(size, crop, threshold, source=0, exposure=exposure)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))

def make_test_meta(home_dir, meta_file, model_meta_file,  
                   r = [0, 1, 0, 0, 0, 0],
                   kp = [0.5, 1, 0, 0, 0, 0.5],
                   ki = [0.3, 0, 0, 0, 0, 0.1],
                   ei_bnd = [[-5, 0, 0, 0, 0, -45], [5, 0, 0, 0, 0, 45]],
                   num_steps = 1000,                   
#                   work_frame = [155, 392, 92.8-3, 180, 0, 180], # circle
                   work_frame = [355, 70, 40-3, 180, 0, 180], # spiral
#                   work_frame = [375, 205, 40-3, 180, 0, 180], # volute
#                   work_frame = [10, 392, 89-3, 180, 0, 180], # clover
#                   work_frame = [-100, 320, 89-3, 180, 0, 180], # teardrop
                   ):
    test_dir =  os.path.dirname(meta_file)
        
    # load model metadata
    with open(os.path.join(home_dir, model_meta_file), 'r') as f: 
        meta = json.load(f)  
    test_image_dir = os.path.join(test_dir, 'images_bw')
    test_df_file = os.path.join(test_dir, 'predictions.csv')
    size = tuple(meta['size']) # ensure appropriate type
    
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['model_meta_file'], meta['test_dir']
    return meta

        
def control_dnn(robot, sensor, model, test_image_dir, target_names, num_frames, 
                robot_tcp, base_frame, home_pose, work_frame, linear_speed, angular_speed,         
                num_steps = 10,
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-np.inf,]*6, [np.inf,]*6],
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
        test_file = os.path.join(test_image_dir, f'image_{i+1}_0.png')
            
        # control signal
        r_q = euler2quat(r, axes='rxyz')
        y_q = euler2quat(-y[i,], axes='rxyz')           
        e[i,] = quat2euler(inv_transform(r_q, y_q), axes='rxyz')
        ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,], ei_bnd[0]), ei_bnd[1])
        u[i+1,] = kp*e[i,] + ki*ei[i+1,]

        # transform control signal to sensor frame                
        u_q = euler2quat(u[i+1,], axes='rxyz')
        v_q = euler2quat(v[i,], axes='rxyz')
        v[i+1,] = quat2euler(inv_transform(u_q, v_q), axes='rxyz')
        
        # stopping conditions
        rms_poses = np.mean(np.square(v[i+1,:3] - target_poses[:,:3]), axis=1)
        delta = target_poses[np.argmin(rms_poses),:] - v[i+1,:] 
        if np.any(delta<target_rng[0]): print(f'error: delta={delta}<target={target_rng[0]}\n'.replace('. ','')); break
        if np.any(delta>target_rng[1]): print(f'error: delta={delta}>target={target_rng[1]}\n'.replace('. ','')); break
        if np.any(robot.joint_angles[5]>245+20): print(f'error: joint={robot.joint_angles[5]}>limit=265\n'); break
        if np.any(robot.joint_angles[5]<-360): print(f'error: joint={robot.joint_angles[5]}<limit=-400\n'); break
#        if np.any(robot.joint_angles[5]>245+20): robot.move_joints(robot.joint_angles - [0,0,0,0,0,360])
#        if np.any(robot.joint_angles[5]<-360): robot.move_joints(robot.joint_angles + [0,0,0,0,0,360])
                     
        # move to new pose
        robot.move_linear(v[i+1,])

        # collect data
        if 'VIRTUAL' in robot.info:
            image = cv2.imread(test_file)
            if image is None: print(f'error: out of data\n'); break
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image = image[np.newaxis, ..., np.newaxis] / 255
        else:
            image = sensor.process(num_frames=1+num_frames, outfile=test_file)
            image = image[1,...][np.newaxis,...] / 255                 
    
        # make prediction
        y[i+1,target_inds] = model.predict(image)    
        with np.printoptions(precision=2, suppress=True):
            print(f'{i+1}: e={e[i,]}'.replace('. ',''))
            print(f'joints={robot.joint_angles}'.replace('. ',''))
            print(f'delta={delta}\n'.replace('. ',''))

        pred_row = pd.Series({'sensor_image': os.path.basename(test_file)})
        for j, jtem in enumerate(pose):
            pred_row['pose_'+str(j+1)] = v[i+1,j]
            pred_row['pred_'+str(j+1)] = y[i+1,j]
            pred_row['error_'+str(j+1)] = delta[j]
        pred_df = pred_df.append(pred_row, ignore_index=True)
        
        # update plot
        fig[0].update(v[:i+1,])
        fig[1].update(image, y[i+1,])
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
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
    model_meta_file = os.path.join('edge2d', 'collect2d_rand_03202159', 'version1', 'models', 'train2d_dnn_opt_03211432', 'meta_22.json')    
#    poses_path = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol', 'servo5d', 'servo5d_dnn_03221711', 'predictions.csv')
    
    # Make test meta
    test_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    meta_file = os.path.join('..', 'TacTip_servocontrol', 'servo2d', test_dirname, 'meta.json')
    meta = make_test_meta(home_dir, meta_file, model_meta_file)
    
    os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))
    with open(os.path.join(home_dir, meta_file), 'w') as f: 
            json.dump(meta, f)
    
    # Paths (reroute to temporary folder)
    test_image_dir = os.path.join(home_dir, meta['test_image_dir'])
    meta['test_image_dir'] = os.path.join(os.environ['TEMPPATH'], test_dirname)
    meta['test_df_file'] = os.path.join(home_dir, meta['test_df_file'])
    meta['model_file'] = os.path.join(home_dir, meta['model_file'])
    
    os.makedirs(meta['test_image_dir'], exist_ok=True)
    
    # target trajectory
#    meta['target_poses'] = pd.read_csv(poses_path).loc[:,'pose_1':'pose_6'].values
#    meta['target_rng'] = [[-20, -20, -3, -15, -15, -np.inf], [20, 20, 10, 15, 15, np.inf]]
    
    # startup/load model and make predictions on test data
    model = CNNmodel()
    model.load_model(**meta)
    
    # Collect data
    with make_robot() as robot, make_sensor(**meta) as sensor:       
        pred_df, fig = control_dnn(robot, sensor, model, **meta)        
    pred_df.to_csv(meta['test_df_file'])
    fig[0].finish(os.path.join(home_dir, os.path.dirname(meta_file), 'contour.png'))         
#    fig[1].finish(os.path.join(home_dir, replay_dir, file, 'frames'))         
#    fig[2].finish(os.path.join(home_dir, replay_dir, file, 'control'))         
    
    # Tidy up
    shutil.make_archive(test_image_dir, 'zip', meta['test_image_dir'])
    shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

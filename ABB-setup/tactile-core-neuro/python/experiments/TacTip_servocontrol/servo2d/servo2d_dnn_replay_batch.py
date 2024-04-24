# -*- coding: utf-8 
  
import os, json, warnings, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np

from core.model.dnn.CNN_model import CNNmodel
from servo2d_dnn import control_dnn

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController


def make_robot(ip='127.0.0.1'):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 

def make_sensor():
    class obj(type):
        def __enter__(cls): return cls
        def __exit__(cls, typ, value, tb): pass
    class sensor(metaclass=obj): pass
    return sensor
       

def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol')
  
    replay_dir = os.path.join('servo2d') # [next...[2]]    
#    replay_dir = os.path.join('servo2d', 'disk', 'rX')
#    replay_dir = os.path.join('servo2d', 'disk', 'rY')
#    replay_dir = os.path.join('servo2d', 'disk', 'rZ')
#    replay_dir = os.path.join('servo2d', 'disk', 'rTheta')
#    replay_dir = os.path.join('servo2d', 'shapes2d'); 
    poses_path = os.path.join(home_dir, 'servo2d', 'servo2d_dnn_03231954', 'poses.csv')    
    
    for file in [next(os.walk(os.path.join(home_dir, replay_dir)))[1][2]]:
        with open(os.path.join(home_dir, replay_dir, file, 'meta.json'), 'r') as f:
            meta = json.load(f)
        meta['linear_speed'], meta['angular_speed'] = (100, 100)
        
        # Paths (reroute to temporary folder)
        test_image_dir = os.path.join(home_dir, meta['test_image_dir'])
        meta['test_image_dir'] = os.path.join(os.environ['TEMPPATH'], file)
        meta['model_file'] = os.path.join(home_dir, 'servo2d', 'model', 'model.h5')      
        
        shutil.unpack_archive(test_image_dir+r'.zip', meta['test_image_dir'])
        os.makedirs(meta['test_image_dir'], exist_ok=True)
        
        # target trajectory
#        poses_path = os.path.join(home_dir, replay_dir, file, 'predictions.csv')
        meta['target_poses'] = pd.read_csv(poses_path).loc[:,'pose_1':'pose_6'].values
#        meta['target_rng'] = [[-20, -20, -3, -15, -15, -np.inf], [20, 20, 10, 15, 15, np.inf]]
        
        # startup/load model and make predictions on test data
        model = CNNmodel()
        model.load_model(**meta)
        
        # Collect data
        with make_robot() as robot, make_sensor() as sensor:       
            pred_df, fig = control_dnn(robot, sensor, model, **meta)       
        pred_df.to_csv(os.path.join(home_dir, meta['test_df_file'].replace('predictions', 'predictions_replay')))
        fig[0].finish(os.path.join(home_dir, replay_dir, file, 'contour'))         
        fig[1].finish(os.path.join(home_dir, replay_dir, file, 'frames'))         
        fig[2].finish(os.path.join(home_dir, replay_dir, file, 'control'))   
        
        # Tidy up
        shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

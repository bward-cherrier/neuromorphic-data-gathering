# -*- coding: utf-8  v  -*-
  
import os, json, warnings, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np

from core.model.cnn.regression_cnn import RegressionCNN
from experiments.TacTip_servocontrol.explore2d.explore_cnn_replay import control_cnn_replay

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

def make_robot(ip='127.0.0.1'):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol')
    replay_dir = os.path.join('explore5d', 'objectSlide')

    for file in next(os.walk(os.path.join(home_dir, replay_dir)))[1]:
        with open(os.path.join(home_dir, replay_dir, file, 'meta.json'), 'r') as f:
            meta = json.load(f)
        meta['work_frame'] = np.array(meta['work_frame']) - [0, 0, 20, 0, 0, 0] 
        meta['linear_speed'], meta['angular_speed'] = (100, 100)
    
        # unpack to temporary folder
        meta['test_image_dir'] = os.path.join(os.environ['TEMPPATH'], 'data')
        shutil.unpack_archive(meta['data_image_dir']+r'.zip', meta['test_image_dir'])
    
        # target trajectory
        poses_path = os.path.join(home_dir, replay_dir, file, 'predictions.csv')
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

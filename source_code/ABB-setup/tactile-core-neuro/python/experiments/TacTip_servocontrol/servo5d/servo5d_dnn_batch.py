# -*- coding: utf-8
  
import os, json, warnings, time, shutil
warnings.filterwarnings("ignore")
import numpy as np
import pandas as pd

from core.model.dnn.CNN_model import CNNmodel
from experiments.TacTip_servocontrol.servo2d.servo2d_dnn import control_dnn

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from core.utils.preproc_video_camera import CvPreprocVideoCamera
from vsp.video_stream import CvImageOutputFileSeq, CvVideoDisplay #, CvimageOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_robot(ip='164.11.72.57'):
    return AsyncRobot(SyncRobot(ABBController(ip=ip))) 

# NB set autoexposure off in amcap; reset all settings amcap; don't use ms camera app; move saturdation to right in amcap (greyscale)
def make_sensor(size=None, crop=None, threshold=None, exposure=-6):
    camera=CvPreprocVideoCamera(size, crop, threshold, source=0, exposure=exposure)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))

def make_test_meta(home_dir, meta_file, model_meta_file,  
                   r = [0, 1, 3, 0, 0, 0],
                   kp = [0.5, 1, 0.5, 0.5, 0.5, 0.5],
                   ki = [0.3, 0, 0.3, 0.1, 0.1, 0.1],
                   ei_bnd = [[-5, 0, -5, -15, -15, -45], [5, 0, 5, 15, 15, 45]],
                   num_steps = 10000,
                   work_frame = [155+12.5, 392-62.5, 92.8-3, 180, 0, 180],
                   ):
    test_dir =  os.path.dirname(meta_file)
    model_dir =  os.path.dirname(model_meta_file)
        
    # load model metadata
    with open(os.path.join(home_dir, model_meta_file), 'r') as f: 
        meta = json.load(f)  
    test_image_dir = os.path.join(test_dir, 'images_bw')
    test_df_file = os.path.join(test_dir, 'predictions.csv')
    size = tuple(meta['size']) # ensure appropriate type
        
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta
        

def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
    model_meta_file = os.path.join('edge5d', 'collect5d_rand_03191010', 'version5', 'models', 'train5d_dnn_opt_03210748', 'meta_28.json')    
    poses_path = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol', 'servo5d', 'servo5d_dnn_03221711', 'predictions.csv')
    
    #name = r'disk\rX'; var1 = 'r'; val1 = [[i, 1, 3, 0, 0, 0] for i in range(-5,6)]
    #name = r'disk\rY'; var1 = 'r'; val1 = [[0, i, 3, 0, 0, 0] for i in [1, 2, 3, 4, 5, 6, 9, 12, 15]]
    #name = r'disk\rZ'; var1 = 'r'; val1 = [[0, 1, v, 0, 0, 0] for v in np.arange(1, 5.25, 0.25)]
    #name = r'disk\rPhi'; var1 = 'r'; val1 = [[0, 1, 3, v, 0, 0] for v in np.arange(-15, 15.25, 2.5)]
    name = r'disk\rPsi'; var1 = 'r'; val1 = [[0, 1, 3, 0, v, 0] for v in np.arange(-7.5, 8, 5)]
    #name = r'disk\rTheta'; var1 = 'r'; val1 = [[0, 1, 3, 0, 0, v] for v in np.arange(35, 60.5, 5)]
    var2 = var1; val2 = val1
    
    #name = 'lid'; var1 = 'work_frame'; val1 = [[-310, -330, 239, 180, 0, 180], [-250, -330, 247, 180, 0, 180], [-195, -330, 243, 180, 0, 180]]
    #var2 = 'robot_tcp'; val2 = [[0, 0, 89.1, 0, 0, -180], [0, 0, 89.1, 0, 0, 180], [0, 0, 89.1, 0, 0, 180]]

    meta_dir = os.path.join('..', 'TacTip_servocontrol', 'servo5d', name)

    for i in range(len(val1)):        
        print(f'\n name={name}: var1={var1}, val1={val1[i]}, var2={var2}, val2={val2[i]}\n')
        
        # Make test meta
        test_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
        meta_file = os.path.join(meta_dir, test_dirname, 'meta.json')
        meta = make_test_meta(home_dir, meta_file, model_meta_file)
        meta[var1] = val1[i]; meta[var2] = val2[i] 
        
        os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))
        with open(os.path.join(home_dir, meta_file), 'w') as f: 
                json.dump(meta, f)
    
        # unpack images to temporary folder
        test_image_dir = os.path.join(home_dir, meta['test_image_dir'])
        meta['test_image_dir'] = os.path.join(os.environ['TEMPPATH'], test_dirname)
        meta['test_df_file'] = os.path.join(home_dir, meta['test_df_file'])
        meta['model_file'] = os.path.join(home_dir, meta['model_file'])
    
        # target trajectory
        meta['target_poses'] = pd.read_csv(poses_path).loc[:,'pose_1':'pose_6'].values
        meta['target_rng'] = [[-20, -20, -3-3, -35, -35, -np.inf], [20, 20, 10, 35, 35, np.inf]]
            
        # startup/load model and make predictions on test data
        model = CNNmodel()
        model.load_model(**meta)
        
        # Collect data
        with make_robot() as robot, make_sensor(**meta) as sensor:       
            pred_df, fig = control_dnn(robot, sensor, model, **meta)   
        pred_df.to_csv(meta['test_df_file'])
        fig[0].finish(os.path.join(home_dir, os.path.dirname(meta_file), 'contour.png'))  
        
        # Tidy up
        shutil.make_archive(test_image_dir, 'zip', meta['test_image_dir'])
        shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

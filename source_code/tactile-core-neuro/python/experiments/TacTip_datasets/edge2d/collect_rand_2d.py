# -*- coding: utf-8 -*-
    
import os, time, json, shutil
from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController
from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor
from experiments.TacTip_datasets.collect_rand import make_meta, make_video_df, collect

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
    

def main():    
    # Specify directories and files
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5d')
    meta_file = os.path.join(home_dir, 
         os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')

    # make meta data and dataframe
    meta = make_meta(meta_file,
                     poses_rng = [[-6, 0, 2, 0, 0, -45], [6, 0, 4, 0, 0, 45]], 
                     moves_rng = [[-5, -5, 0, -5, -5, -5], [5, 5, 0, 5, 5, 5]], 
                     num_poses = 2000)    
    make_video_df(**meta)
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)   
    
    # temporary image folder
    temp_meta = {**meta, 'video_dir': os.environ['TEMPPATH'] + r'\videos'+time.strftime('%m%d%H%M')}
    
    # collect data
    collect(**temp_meta)
    
    # tidy up
    shutil.make_archive(meta['video_dir'], 'zip', temp_meta['video_dir'])
    shutil.rmtree(temp_meta['video_dir'])
               
if __name__ == '__main__':
    main()

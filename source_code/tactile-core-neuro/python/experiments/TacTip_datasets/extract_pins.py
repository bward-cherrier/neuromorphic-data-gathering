# -*- coding: utf-8 -*-

import os, json, shutil, time
import pandas as pd
import numpy as np
import scipy.io
from core.sensor.tactile_replay import TactileReplay as tactile_replay

# Specify directories and files - to edit
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5dTap')
meta_file = os.path.join(home_dir, 'collect_tap_rand_5d_11281815', 'meta.json')


def update_meta(meta_file):
    data_dir = os.path.dirname(meta_file)
    with open(meta_file, 'r') as f: meta = json.load(f)  
    
    # update metadata
    video_dir = os.path.join(data_dir, 'videos')
    video_df_file = os.path.join(data_dir, 'targets_video.csv')
    pins_dir = os.path.join(data_dir, 'pins')
    pins_df_file = os.path.join(data_dir, 'targets_pins.csv')

    min_threshold = 100
    max_threshold = 203
    min_area = 24
    max_area = 117
    min_circularity = 0.54
    min_convexity = 0.35
    min_inertia_ratio = 0.21
    tracking = True
    
    # return metadata
    meta.update(locals().copy())
    del meta['data_dir'], meta['meta'], meta['f'],
    return meta

def extract_pins(sensor, pins_dir, pins_df_file, video_dir, video_df_file, **kwargs):   
    os.mkdir(pins_dir)
    video_df = pd.read_csv(video_df_file)
    pins_df = pd.DataFrame()
    
    video_filenames = [f for f in os.listdir(video_dir) if f.endswith('.mp4')]
    for i, video_filename in enumerate(video_filenames):
        pins_filename = video_filename.replace('video','pins').replace('mp4','mat')
        video_file = os.path.join(video_dir, video_filename)
        pins_file = os.path.join(pins_dir, pins_filename)
    
        pins_df_row = video_df[video_df['sensor_video'] == video_filename]
        pins_df_row = pins_df_row.rename(columns={'sensor_video' : 'sensor_pins'})
        pins_df_row['sensor_pins'] = pins_filename
        pins_df = pins_df.append(pins_df_row)
        
        scipy.io.savemat(pins_file, {'pins': sensor.replay_pins(video_file)})
        if (i+1) % 10 == 0: 
            print('Processed video {} of {}'.format(i+1, len(video_filenames)))

    pins_df = pins_df.reset_index(drop=True)
    pins_df.to_csv(pins_df_file, index=False)


def main(meta_file):        
    meta = update_meta(meta_file)
    with open(meta_file, 'w') as f: json.dump(meta, f)
    
    temp_meta = {**meta, 'video_dir': os.environ['TEMPPATH'] + r'\videos'+time.strftime('%m%d%H%M'),
                         'pins_dir': os.environ['TEMPPATH'] + r'\pins'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['video_dir']+r'.zip', temp_meta['video_dir'])
       
    sensor = tactile_replay('cameraDisp', False, **temp_meta)
    init_pins = sensor.init_pins(temp_meta['video_dir'] + r'\video_init.mp4')
    sensor.set_pins(np.array(init_pins))
    
    extract_pins(sensor, **temp_meta)
    
    shutil.make_archive(meta['pins_dir'], 'zip', temp_meta['pins_dir'])
    shutil.rmtree(temp_meta['video_dir'])
    shutil.rmtree(temp_meta['pins_dir'])

if __name__ == '__main__':
    main(meta_file)

# -*- coding: utf-8 -*-

import os, json, shutil, time
from core.model.cnn.cnn_utils import extract_video_frames
from experiments.TacTip_datasets.extract_images import process_image_dir


def update_meta(meta_file,
                offset = 0,
                size = (128, 128),
                crop = (160, 70, 490, 400),
                threshold = (11, -5),
                ):
    data_dir = os.path.dirname(meta_file)
    with open(meta_file, 'r') as f: meta = json.load(f)  
    
    # update metadata
    video_dir = os.path.join(data_dir, 'videos')
    video_df_file = os.path.join(data_dir, 'predictions.csv')
    image_dir = os.path.join(data_dir, 'images_bw')
    image_df_file = os.path.join(data_dir, 'predictions_images.csv')
    
    # return metadata
    meta.update(locals().copy())
    del meta['data_dir'], meta['meta'], meta['f'],
    return meta


def main():    
    # Specify directories and files 
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol\\explore3d\\headSlide')
    meta_file = os.path.join(home_dir, 'explore3dCNNBatch08222001', 'meta.json')

    # Make meta data
    meta = update_meta(meta_file)
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)
    
    # Images in temporary folder
    temp_meta = {**meta, 'video_dir': os.environ['TEMPPATH'] + r'\videos'+time.strftime('%m%d%H%M'),
                         'image_dir': os.environ['TEMPPATH'] + r'\images'+time.strftime('%m%d%H%M'),
                         'pp_image_dir': os.environ['TEMPPATH'] + r'\pp_images'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['video_dir']+r'.zip', temp_meta['video_dir'])
    
    # Extract videos and processs images
    extract_video_frames(**temp_meta)
    process_image_dir(**temp_meta)
    
    # Tidy up
    shutil.make_archive(meta['image_dir'], 'zip', temp_meta['pp_image_dir'])
    shutil.rmtree(temp_meta['video_dir'])
    shutil.rmtree(temp_meta['image_dir'])
               
if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-

import os, json, shutil, time, cv2
import numpy as np
from core.model.cnn.cnn_utils import extract_video_frames


def update_meta(meta_file,
                offset = 1,
                size = (128, 128),
                crop = (160, 70, 490, 400),
                threshold = (11, -5),
                ):
    data_dir = os.path.dirname(meta_file)
    with open(meta_file, 'r') as f: meta = json.load(f)  
    
    # update metadata
    video_dir = os.path.join(data_dir, 'videos')
    video_df_file = os.path.join(data_dir, 'targets_video.csv')
    image_dir = os.path.join(data_dir, 'images_bw')
    image_df_file = os.path.join(data_dir, 'targets_image.csv')
    
    # return metadata
    meta.update(locals().copy())
    del meta['data_dir'], meta['meta'], meta['f'],
    return meta

# function for pre-processing images
def process_image(image, crop=None, size=None, threshold=None, **kwargs): 
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if crop is not None:
        x0, y0, x1, y1 = crop
        image = image[y0:y1, x0:x1]
    if threshold is not None:
        image = image.astype('uint8')
        image = cv2.medianBlur(image, 5)
        width, offset = threshold
        image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,                                       
                                      cv2.THRESH_BINARY, width, offset)
    if size is not None:
        image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    image = image[..., np.newaxis]  
    return image

def process_image_dir(image_dir, pp_image_dir, crop=None, size=None, threshold=None, **kwargs):
    if not os.path.isdir(pp_image_dir):
        os.makedirs(pp_image_dir)
        image_files = (f for f in os.listdir(image_dir) if f.endswith('.jpg'))
        for image_file in image_files:
            image = cv2.imread(os.path.join(image_dir, image_file))
            image = process_image(image, crop, size, threshold)
            cv2.imwrite(os.path.join(pp_image_dir, image_file), image);
    return pp_image_dir


def main():    
    # Specify directories and files 
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\plane3dTap')
    meta_file = os.path.join(home_dir, 'collect_tap_rand_efi_01211541', 'meta.json')

    # Make meta data
    meta = update_meta(meta_file)
#    meta = update_meta(meta_file, size=None, crop=None, threshold=None)
    
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

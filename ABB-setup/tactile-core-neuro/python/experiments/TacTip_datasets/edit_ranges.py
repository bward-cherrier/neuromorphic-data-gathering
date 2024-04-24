# -*- coding: utf-8 -*-

import os, json
import pandas as pd

home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
meta_file = os.path.join('edge2d', 'collect_rand_2d_03210705', 'meta.json')

image_dir = meta_file.replace('meta.json','images_bw')
image_df_file = meta_file.replace('meta.json','targets_image.csv')

# save directories
save_dir = os.path.join(os.path.dirname(meta_file), 'version0')
meta_file_new = os.path.join(save_dir, 'meta.json')
image_df_file_new = os.path.join(save_dir, 'targets_image.csv')

# open
with open(os.path.join(home_dir, meta_file), 'r') as f: 
    meta = json.load(f) 
image_df = pd.read_csv(os.path.join(home_dir, image_df_file))

# define new ranges
poses_rng = meta['poses_rng']
#poses_rng[0][0] = -4
#poses_rng[0][2] = 2
#poses_rng[1][0] = 4

# select by range
#lower = image_df[['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']]>=poses_rng[0]
#upper = image_df[['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']]<=poses_rng[1]
#inds = pd.concat([lower, upper], axis=1).all(axis=1)
#inds = pd.concat([inds, image_df['pose_id']%2!=0], axis=1).all(axis=1)

# select by even/odd
#inds = image_df['pose_id']%2!=0

# selection
#image_df = image_df[inds]
meta['meta_file'] = meta_file_new
meta['poses_rng'] = poses_rng
meta['num_poses'] = len(image_df) 
meta['image_dir'] = image_dir
meta['image_df_file'] = image_df_file_new

os.makedirs(os.path.join(home_dir, save_dir), exist_ok=True) 

with open(os.path.join(home_dir, meta_file_new), 'w') as f: 
    json.dump(meta, f) 
image_df.to_csv(os.path.join(home_dir, image_df_file_new), index=False)

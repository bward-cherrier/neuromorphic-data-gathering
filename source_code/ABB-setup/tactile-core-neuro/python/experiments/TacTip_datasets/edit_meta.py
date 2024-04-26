# -*- coding: utf-8 -*-

import os, json

#def make_meta(meta_file):               
#    with open(meta_file, 'r') as f: 
#        meta = json.load(f)
#    
##    robot_tcp = [0, 0, 89.1, 0, 0, 0]
##    base_frame = [0, 0, 0, 0, 0, 0]
##    home_pose = [400, 0, 300, 180, 0, 180]
##    work_frame = meta['work_frame']
##    linear_speed = meta['robot_speed'][0]
##    angular_speed = meta['robot_speed'][1]
##    num_frames = meta['n_frames']
##   
##    try: 
##        tap_move = [meta['tap_traj'], [0, 0, 0, 0, 0, 0]]
##    except:
##        pass
##        
##    poses_rng = [[], []]
##    meta['null'] = [0, 0]
##    for item in ['r_x', 'null', 'null', 'null', 'null', 'null']:
##        poses_rng[0].append(meta[item][0])
##        poses_rng[1].append(meta[item][1])
##    
##    moves_rng = [[], []]
##    try:
##        for item in ['r_x_move', 'r_y_move', 'null', 'r_roll_move', 'r_pitch_move', 'r_yaw_move']:
##            moves_rng[0].append(meta[item][0])
##            moves_rng[1].append(meta[item][1])                 
##        del meta['null']
##    except:
##        pass
##    
##    obj_poses = [meta['objs']]
##    num_poses = meta['n_poses']
## 
##    try:
##        video_dir = meta['video_dir']
##        video_df_file = meta['video_target_file']
##        image_dir = meta['image_dir']
##        image_df_file = meta['image_target_file']
##    except:
##        pass
##
##    try:
##        pins_dir = meta['pins_dir']
##        pins_df_file = meta['pins_target_file']
##        pins_init_file = meta['pins_init_file']
##    except:
##        pass
#
##    try:
##        offset = meta['offset']
##        size = meta['dims']
##        crop = meta['bbox']
##        threshold = meta['threshold']
##
##        num_conv_layers = len(meta['conv_filters'])
##        num_conv_filters = meta['conv_filters'][0]
##        num_dense_layers = len(meta['dense_units'])-1
##        num_dense_units = meta['dense_units'][0]
##        activation = meta['activation']
##        dropout = meta['dropout']
##        kernel_l1 = meta['kernel_reg_l1']
##        kernel_l2 = meta['kernel_reg_l2']
##        batch_size = meta['batch_size']
##
##        epochs = meta['epochs']
##        patience = meta['patience']
##        lr = meta['learning_rate']
###        epsilon = meta['epsilon']
##        decay = meta['learning_rate_decay']
##        target_names = meta['target_names']      
##        
###        valid_split = meta['validation_split']
###        conv_filters = meta['conv_filters']
###        dense_units = meta['dense_units']
###        batch_norm = meta['batch_norm']
##        
##        model_file = meta['model_file']
##        train_image_dir = meta['image_dir']
##        train_df_file = meta['image_target_file']
##        valid_image_dir = meta['validation_image_dir']
##        valid_df_file = meta['validation_target_file']    
##        
##        del video_dir, video_df_file, image_dir, image_df_file
##        del pins_dir, pins_df_file, pins_init_file
##    except:
##        pass     
#    
##    meta_new = locals().copy()
##    del meta_new['meta'], meta_new['item'], meta_new['f']
##    with open(meta_file+'1', 'w') as f: 
##        json.dump(meta_new, f)
##    print(meta_new)   
# 
### model dir
###def main(home_dir):    
##home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5dTap\collectTapRand5d06200640\models')
##for data_dir_name in os.listdir(home_dir):
##    for (dir_path, dir_names, file_names) in os.walk(home_dir):
##        for file_name in file_names:
##            meta_file = os.path.join(dir_path, file_name)
##            try:
##                make_meta(meta_file)
###                os.remove(meta_file+'1')
##            except:
##                pass
#    
#    home_dir = meta['meta_file'].split('edge5d\\')[0]
#    meta_new = {'home_dir': home_dir[:-1], **meta}
#    for key in ['meta_file', 'model_file', 'valid_image_dir', 'train_image_dir', 'train_df_file', 'valid_df_file', 'test_df_file', 'test_image_dir', 'image_df_file', 'image_dir', 'video_dir', 'video_df_file']:
#        try:
#            print(meta_new[key])   
#            meta_new[key] =  meta[key].split(home_dir)[1]
#        except:
#            pass
#        
#    with open(meta_file, 'w') as f: 
#        json.dump(meta_new, f)
#    os.remove(meta_file+'1')

## data dir only
#home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', r'edge5d')
#for (dirpath, dirnames, filenames) in os.walk(home_dir):
#    for filename in filenames:
#        if filename.endswith('.json'):
#            try:
#                make_meta(os.path.join(dirpath, filename))
#            except:
#                pass


home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'plane3d')
for (dirpath, dirnames, filenames) in os.walk(home_dir):
    for filename in filenames:
        if filename.endswith('.json'):
            print(os.path.join(dirpath, filename))
            fin = open(os.path.join(dirpath, filename), 'rt')
            data = fin.read()
            data = data.replace('collect_rand_3d', 'collect3d_rand')
#            data = data.replace('test_dnn_batch_3d', 'test3d_dnn_batch')
#            data = data.replace('train_dnn_opt_3d', 'train3d_dnn_opt')
            fin.close()
            
            fin = open(os.path.join(dirpath, filename), 'wt')
            fin.write(data)
            fin.close()

#if __name__ == '__main__':
#    main(home_dir)

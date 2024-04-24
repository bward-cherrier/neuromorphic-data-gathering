# -*- coding: utf-8 -*-

import os, json
from shutil import copyfile

def make_meta(meta_file):               
    with open(meta_file, 'r') as f: 
        meta = json.load(f)
    
    robot_tcp = [0, 0, 89.1, 0, 0, -180] #-360 some cases
    base_frame = [0, 0, 0, 0, 0, 0]
    home_pose = [400, 0, 300, 180, 0, 180]
    work_frame = meta['work_frame']
    linear_speed = meta['robot_speed'][0]
    angular_speed = meta['robot_speed'][1]
    num_frames = meta['n_frames']
   
#    try: 
#        tap_move = [meta['tap_traj'], [0, 0, 0, 0, 0, 0]]
#    except:
#        pass
        
    poses_rng = [[], []]
    meta['null'] = [0, 0]
    for item in ['r_x', 'null', 'r_z', 'null', 'null', 'r_yaw']:
        poses_rng[0].append(meta[item][0])
        poses_rng[1].append(meta[item][1])
    
    moves_rng = [[], []]
    try:
        for item in ['r_x_move', 'r_y_move', 'null', 'null', 'null', 'r_yaw_move']:
            moves_rng[0].append(meta[item][0])
            moves_rng[1].append(meta[item][1])                 
        del meta['null']
    except:
        pass
    
    obj_poses = [meta['objs']]
    num_poses = meta['n_poses']

    offset = meta['offset']
    size = meta['dims']
    crop = meta['bbox']
    threshold = meta['threshold']

    num_conv_layers = len(meta['conv_filters'])
    num_conv_filters = meta['conv_filters'][0]
    num_dense_layers = len(meta['dropout'])-num_conv_layers
    try: 
        num_dense_units = meta['dense_units'][0]
    except: 
        num_dense_units = meta['dense_units']     
    activation = meta['activation']
    dropout = meta['dropout'][-1]
    kernel_l1 = meta['kernel_reg_l1'][-1]
    kernel_l2 = meta['kernel_reg_l2'][-1]
    batch_size = meta['batch_size']
    batch_norm = meta['batch_norm'][-1] 
    # needs checking - may have batch_norm false in all models - no need for batch_size

    epochs = meta['epochs']
    patience = meta['patience']
    lr = meta['learning_rate']
    decay = meta['learning_rate_decay']
    target_names = meta['target_names']      
    
    try:
        model_file = meta['model_file']
    except:
        model_file = meta['best_model_file']    
    
    model_file = r'D:\OneDrive - University of Bristol\Data onedrive\TacTip_servocontrol\explore2d\train2dCNN07082051\model.h5'
    
    train_image_dir = meta['image_dir']
    train_df_file = meta['image_target_file']
    try:
        valid_image_dir = meta['validation_image_dir']
    except:
        pass
    try:
        valid_df_file = meta['validation_target_file']    
    except:
        pass
    try: 
        valid_split = meta['validation_split']
    except:
            pass
    
    r = meta['r']
    kp = meta['kp']
    ki = meta['ki']
    ei_bnd = meta['eiBnd']
    num_steps = meta['n_steps']

    data_image_dir = os.path.join(os.path.dirname(meta_file), 'imagesPython')
#    print(meta)
    
    meta_new = locals().copy()
    del meta_new['meta'], meta_new['item'], meta_new['f']
    with open(meta_file, 'w') as f: 
        json.dump(meta_new, f)
    print(meta_new)   
 
## model dir
##def main(home_dir):    
#home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5dTap\collectTapRand5d06200640\models')
#for data_dir_name in os.listdir(home_dir):
#    for (dir_path, dir_names, file_names) in os.walk(home_dir):
#        for file_name in file_names:
#            meta_file = os.path.join(dir_path, file_name)
#            try:
#                make_meta(meta_file)
#            except:
#                pass

# data dir only
home_dir = os.path.join(os.environ['DATAPATH'], r'TacTip_servocontrol\explore2d')
for (root, dirs, files) in os.walk(home_dir):
    for data_dir_name in dirs: 
        meta_file = os.path.join(root, data_dir_name, 'meta.json')
        try:
            copyfile(meta_file, meta_file.replace('meta.json', 'meta_old.json'))
            make_meta(meta_file)
            print(meta_file)
        except:
            pass
        
            
#if __name__ == '__main__':
#    main(home_dir)

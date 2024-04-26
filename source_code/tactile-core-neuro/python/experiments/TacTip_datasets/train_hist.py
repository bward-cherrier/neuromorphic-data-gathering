# -*- coding: utf-8 -*-

import os, time, json, shutil

from core.model.hist.classification_hist import ClassificationHist

# Specify directories and files
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip-datasets\edge5dTap')
train_meta_file = os.path.join(home_dir, 'collect_tap_rand_5d_11281815', 'meta.json')

def make_model_meta(meta_file, train_meta_file):
    model_dir = os.path.dirname(meta_file)
    train_dir = os.path.dirname(train_meta_file)
    os.makedirs(model_dir)
   
    # load metadata
    with open(train_meta_file, 'r') as f: meta = json.load(f)    
        
    # update metadata
    model_file = os.path.join(model_dir, 'model.mat')
    train_pins_dir = os.path.join(train_dir, 'pins')
    train_df_file = os.path.join(train_dir, 'targets_pins.csv')       
    
    target_names = ['pose_1', 'pose_3', 'pose_4', 'pose_5', 'pose_6']
    num_bins = 40
    offset = 0#-2
    
    # save metadata in model dir
    meta.update(locals().copy())
    del meta['video_dir'], meta['video_df_file'], meta['pins_dir'], meta['pins_df_file'] 
    del meta['meta'], meta['f'], meta['train_dir'], meta['model_dir'], meta['train_meta_file'] 
    try: del meta['image_dir'], meta['image_df_file'] 
    except: pass
    with open(meta_file, 'w') as f: json.dump(meta, f)
    return meta


def main(train_meta_file):
    meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', 
        os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')
      
    meta = make_model_meta(meta_file, train_meta_file)
    temp_meta = {**meta, 'train_pins_dir': os.environ['TEMPPATH'] + r'\train'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['train_pins_dir']+r'.zip', temp_meta['train_pins_dir'])
       
    hist = ClassificationHist(**temp_meta)
    hist.fit_from_file(**temp_meta)
    hist.close()
    
    shutil.rmtree(temp_meta['train_pins_dir'])

if __name__ == '__main__':
    main(train_meta_file)

# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.dnn.CNN_model import CNNmodel


def make_test_meta(home_dir, meta_file, test_meta_file, model_meta_file):
    model_dir =  os.path.dirname(model_meta_file)
        
    # load test metadata
    with open(os.path.join(home_dir, test_meta_file), 'r') as f: 
        meta = json.load(f)    
    test_image_dir = meta['image_dir']         
    test_df_file = meta['image_df_file']
    
    # load model metadata
    with open(os.path.join(home_dir, model_meta_file), 'r') as f: 
        meta = json.load(f)
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
    size = tuple(meta['size'])     # ensure appropriate type

    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['test_meta_file'], meta['model_meta_file']
    del meta['model_dir']
    return meta

def plot_pred(pred_df, target_names, model_file, meta_file, poses_rng, **kwargs):
    plt.rcParams.update({'font.size': 18})
    n = len(target_names)
    
    fig, axes = plt.subplots(ncols=n, figsize=(7*n, 7))
    fig.suptitle(model_file.replace(os.environ['DATAPATH'],'') + '\n' + 
                 os.path.dirname(meta_file.replace(os.environ['DATAPATH'],'')))
    fig.subplots_adjust(wspace=0.3)
    n_smooth = int(pred_df.shape[0]/20)    
    for i, ax in enumerate(axes): 
        sort_df = pred_df.sort_values(by=['target_'+str(i+1)])
        ax.scatter(sort_df['target_'+str(i+1)], sort_df['pred_'+str(i+1)], s=1, 
                   c=sort_df['target_1'], cmap='inferno')
        ax.plot(sort_df['target_'+str(i+1)].rolling(n_smooth).mean(), 
                sort_df['pred_'+str(i+1)].rolling(n_smooth).mean(), c='red')
        ax.set(xlabel='target '+target_names[i], ylabel='predicted '+target_names[i])
        ind = int(target_names[i][-1])-1
        ax.set_xlim(poses_rng[0][ind], poses_rng[1][ind])
        ax.set_ylim(poses_rng[0][ind], poses_rng[1][ind])
        ax.text(0.05,0.9, 'MAE='+str(sort_df['error_'+str(i+1)].mean())[0:4], transform=ax.transAxes)    
        ax.grid(True)
    return fig

def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'plane3d')
    test_meta_file = os.path.join('plane3d', 'collect3d_rand_03201334', 'meta.json')
    model_meta_file = os.path.join('plane3d', 'collect3d_rand_03192000', 'version1', 'models', 'train3d_dnn_opt_03212140', 'meta.json')
    
    # Make test meta
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, test_meta_file, model_meta_file)
    
    os.makedirs(os.path.dirname(meta_file), exist_ok=True) 
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)    
    
    temp_dir = os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))
    shutil.unpack_archive(os.path.join(home_dir, meta['test_image_dir']+r'.zip'), os.path.join(temp_dir, meta['test_image_dir']))
    
    meta['model_file'] = os.path.join(home_dir, meta['model_file'])
    meta['test_image_dir'] = os.path.join(temp_dir, meta['test_image_dir'])
    meta['test_df_file'] = os.path.join(home_dir, meta['test_df_file'])
        
    # startup/load model and make predictions on test data
    cnn = CNNmodel()
    cnn.load_model(meta)
    cnn.print_model_summary()
    pred = cnn.predict_from_file(meta)
    
    # analyze predictions
    pred_df = pd.read_csv(meta['test_df_file'])
    for i, item in enumerate(meta['target_names'], start=1):
        pred_df['pred_'+str(i)] = pred[:, i-1]
        pred_df['target_'+str(i)] = pred_df[item]
        pred_df['error_'+str(i)] = abs(pred_df['pred_'+str(i)] - pred_df['target_'+str(i)])
    pred_df.to_csv(os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname, 'predictions.csv'))
    fig = plot_pred(pred_df, **meta)
    fig.savefig(os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname, 'errors.png'), bbox_inches='tight')   
    
    # tidy up
    shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

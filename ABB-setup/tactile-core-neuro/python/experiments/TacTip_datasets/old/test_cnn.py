# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.cnn.regression_cnn import RegressionCNN

# Specify directories and files - to edit
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5dTap')
test_meta_file = os.path.join(home_dir, 'collectTapRand5d06200642\meta.json')
model_meta_file = os.path.join(home_dir, r'collectTapRand5d06200640\models\trainCNNopt_11260921\meta1.json1')


def make_test_meta(meta_file, test_meta_file, model_meta_file):
    test_dir =  os.path.dirname(test_meta_file)
    model_dir =  os.path.dirname(model_meta_file)
        
    # load metadata
    with open(model_meta_file, 'r') as f: 
        meta = json.load(f)

    # update metadata
    test_image_dir = os.path.join(test_dir, 'images')
    test_df_file = os.path.join(test_dir, 'targets_image.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
    
    # ensure appropriate type 
    crop = tuple(meta['crop'])    
    size = tuple(meta['size']) 
    threshold = tuple(meta['threshold'])
        
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['test_meta_file'], meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta

def plot_pred(pred_df, target_names, model_file, meta_file, poses_rng, **kwargs):
    plt.rcParams.update({'font.size': 18})
    n = len(target_names)
    
    fig, axes = plt.subplots(ncols=n, figsize=(7*n, 7))
    fig.suptitle(model_file.replace(os.environ['DATAPATH'],'') + '\n' + 
                 os.path.dirname(meta_file.replace(os.environ['DATAPATH'],'')))
    fig.subplots_adjust(wspace=0.3)
    n_smooth = int(pred_df.shape[0]/20)    
    for i, item in enumerate(target_names): 
        sort_df = pred_df.sort_values(by=['target_'+str(i+1)])
        sort_df.plot(ax=axes[i], x='target_'+str(i+1), y='pred_'+str(i+1), 
                     s=1, c='black', kind='scatter', grid=True)
        axes[i].plot(sort_df['target_'+str(i+1)].rolling(n_smooth).mean(), 
                    sort_df['pred_'+str(i+1)].rolling(n_smooth).mean(), c='red')
        axes[i].set(xlabel='target '+item, ylabel='predicted '+item)
        ind = int(item[-1])-1
        axes[i].set_xlim(poses_rng[0][ind], poses_rng[1][ind])
        axes[i].set_ylim(poses_rng[0][ind], poses_rng[1][ind])
        axes[i].text(0.05, 0.9, 'MAE='+str(sort_df['error_'+str(i+1)].mean())[0:4], 
                                transform=axes[i].transAxes)    
    return fig
    

def main(test_meta_file, model_meta_file):
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, test_meta_file, model_meta_file)
    
    os.makedirs(os.path.dirname(meta_file), exist_ok=True)
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)    

    # unpack to temporary folder    
    temp_meta = {**meta, 'test_image_dir': os.environ['TEMPPATH'] + r'\test'+time.strftime('%m%d%H%M')}           
    shutil.unpack_archive(meta['test_image_dir']+r'.zip', temp_meta['test_image_dir'])
    
    # startup/load model and make predictions on test data
    cnn = RegressionCNN()
    cnn.load_model(**temp_meta)
    cnn.print_model_summary()
    pred = cnn.predict_from_file(**temp_meta)
    
    # analyze predictions
    pred_df = pd.read_csv(temp_meta['test_df_file'])
    for i, item in enumerate(temp_meta['target_names'], start=1):
        pred_df['pred_'+str(i)] = pred[:, i-1]
        pred_df['target_'+str(i)] = pred_df[item]
        pred_df['error_'+str(i)] = abs(pred_df['pred_'+str(i)] - pred_df['target_'+str(i)])
    pred_df.to_csv(os.path.dirname(meta_file) +r'\predictions.csv')
    fig = plot_pred(pred_df, **meta)
    fig.savefig(os.path.dirname(meta_file) + r'\errors.png', bbox_inches='tight')   
    
    shutil.rmtree(temp_meta['test_image_dir'])

if __name__ == '__main__':
    main(test_meta_file, model_meta_file)

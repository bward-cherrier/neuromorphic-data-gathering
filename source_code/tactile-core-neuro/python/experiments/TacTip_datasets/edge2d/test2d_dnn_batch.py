# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.dnn.CNN_model import CNNmodel
from test2d_dnn import plot_pred, make_test_meta


def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
    test_meta_file = os.path.join('edge2d', 'collect2d_rand_03210705', 'version0', 'meta.json')
    model_dir = os.path.join('edge2d', 'collect2d_rand_03202159', 'version1', 'models', 'train2d_dnn_opt_03211432')
    
    test_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    test_dir = os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname) 
    os.makedirs(os.path.join(home_dir, test_dir), exist_ok=True)     
    
    mean_df = pd.DataFrame()
    trials_df = pd.read_csv(os.path.join(home_dir, model_dir, 'trials.csv'))
    trials_rng = trials_df.sort_values(by='loss')['trial'].tolist()
    
    for j, trial in enumerate(trials_rng[:5]):
        meta_file = os.path.join(test_dir, 'meta_'+str(j+1)+'.json')
        model_meta_file = os.path.join(model_dir, 'meta_'+str(trial)+'.json')
        meta = make_test_meta(home_dir, meta_file, test_meta_file, model_meta_file)
        with open(os.path.join(home_dir, meta_file), 'w') as f: 
            json.dump(meta, f)  
        
        temp_dir = os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))
        shutil.unpack_archive(os.path.join(home_dir, meta['test_image_dir']+r'.zip'), os.path.join(temp_dir, meta['test_image_dir']))
    
        meta['model_file'] = os.path.join(home_dir, meta['model_file'])
        meta['test_image_dir'] = os.path.join(temp_dir, meta['test_image_dir'])
        meta['test_df_file'] = os.path.join(home_dir, meta['test_df_file'])
        
        try:
            # startup/load model and make predictions on test data
            cnn = CNNmodel()
            cnn.load_model(**meta)
            pred = cnn.predict_from_file(**meta)
        
            # analyze predictions
            pred_df = pd.read_csv(meta['test_df_file'])
            for i, item in enumerate(meta['target_names'], start=1):
                pred_df['pred_'+str(i)] = pred[:, i-1]
                pred_df['target_'+str(i)] = pred_df[item]
                pred_df['error_'+str(i)] = abs(pred_df['pred_'+str(i)] - pred_df['target_'+str(i)])
            pred_df.to_csv(os.path.join(home_dir, test_dir, 'pred_'+str(j+1)+'.csv'))    
            fig = plot_pred(pred_df, **meta)
            fig.savefig(os.path.join(home_dir, test_dir, 'error_'+str(j+1)+'.jpg'), bbox_inches='tight')   
            
            mean_df = mean_df.append([pred_df.mean()])
        except:
            mean_df = mean_df.append([None])
        shutil.rmtree(meta['test_image_dir'])
    
    # Summary plot    
    n = len(meta['target_names'])
    fig = plt.figure(figsize=(7*n, 7))
    fig.suptitle(os.path.dirname(meta['model_file']).replace(os.environ['DATAPATH'],'') + '\n' + 
                 os.path.dirname(meta['meta_file']).replace(os.environ['DATAPATH'],''))
    fig.subplots_adjust(wspace=0.3)
    for i, item in enumerate(meta['target_names']):
        ax = fig.add_subplot(1, n, 1+i)
        ax.scatter(range(1,len(mean_df.index)+1), mean_df['error_'+str(i+1)], c='black')
        ax.grid(True)
        ax.set(xlabel='model', ylabel='MAE '+item)
        ax.set_ylim(bottom=0)
    fig.savefig(os.path.join(home_dir, test_dir, 'summary.png'), bbox_inches='tight')        

if __name__ == '__main__':
    main()

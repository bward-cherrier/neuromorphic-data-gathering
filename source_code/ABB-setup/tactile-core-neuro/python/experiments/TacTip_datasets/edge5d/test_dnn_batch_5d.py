# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.dnn.CNN_model import CNNmodel
from experiments.TacTip_datasets.test_dnn import make_test_meta, plot_pred

def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5d')
    test_meta_file = os.path.join(home_dir, 'collectRand5d06200645\meta.json')
    model_dir = os.path.join(home_dir, r'collectRand5d06200643\models\train_dnn_opt_5d_12280959')
    
    test_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    test_dir = os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname) 
    os.makedirs(test_dir, exist_ok=True)     
    
    mean_df = pd.DataFrame()
    trials_df = pd.read_csv(os.path.join(model_dir, 'trials.csv'))
    trials_rng = trials_df.sort_values(by='loss')['trial'].tolist()
    
    for j, trial in enumerate(trials_rng[:5]):
        meta_file = os.path.join(test_dir, 'meta_'+str(j+1)+'.json')
        model_meta_file = os.path.join(model_dir, 'meta_'+str(trial)+'.json')
        meta = make_test_meta(meta_file, test_meta_file, model_meta_file)
        with open(meta_file, 'w') as f: 
            json.dump(meta, f)  
        
        temp_meta = {**meta, 'test_image_dir': os.path.join(os.environ['TEMPPATH'], 'test'+time.strftime('%m%d%H%M'))}           
        shutil.unpack_archive(meta['test_image_dir']+r'.zip', temp_meta['test_image_dir'])
    
        try:
            # startup/load model and make predictions on test data
            cnn = CNNmodel()
            cnn.load_model(**temp_meta)
            pred = cnn.predict_from_file(**temp_meta)
        
            # analyze predictions
            pred_df = pd.read_csv(temp_meta['test_df_file'])
            for i, item in enumerate(temp_meta['target_names'], start=1):
                pred_df['pred_'+str(i)] = pred[:, i-1]
                pred_df['target_'+str(i)] = pred_df[item]
                pred_df['error_'+str(i)] = abs(pred_df['pred_'+str(i)] - pred_df['target_'+str(i)])
            pred_df.to_csv(os.path.join(test_dir, 'pred_'+str(j+1)+'.csv'))    
            fig = plot_pred(pred_df, **meta)
            fig.savefig(os.path.join(test_dir, 'error_'+str(j+1)+'.jpg'), bbox_inches='tight')   
            
            mean_df = mean_df.append([pred_df.mean()])
        except:
            mean_df = mean_df.append([None])
        shutil.rmtree(temp_meta['test_image_dir'])
    
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
    fig.savefig(os.path.join(test_dir, 'summary.png'), bbox_inches='tight')        

if __name__ == '__main__':
    main()

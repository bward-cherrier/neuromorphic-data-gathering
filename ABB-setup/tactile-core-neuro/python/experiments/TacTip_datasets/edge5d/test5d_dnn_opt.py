# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
from core.model.dnn.CNN_model import CNNmodel
from test5d_dnn import make_test_meta, plot_pred


def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
    test_meta_file = os.path.join('edge5d', 'collect5d_rand_03201048', 'version2', 'meta.json')
    model_dir = os.path.join('edge5d', 'collect5d_rand_03191010', 'version5', 'models', 'train5d_dnn_opt_03210748')
    
    test_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    test_dir = os.path.join(os.path.dirname(test_meta_file), 'tests', test_dirname) 
    os.makedirs(os.path.join(home_dir, test_dir), exist_ok=True)     
    
    for j in range(200):
        meta_file = os.path.join(test_dir, 'meta_'+str(j+1)+'.json')
        model_meta_file = os.path.join(model_dir, 'meta_'+str(j+1)+'.json')
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
        except:
            pass
                        
        shutil.rmtree(meta['test_image_dir'])

if __name__ == '__main__':
    main()

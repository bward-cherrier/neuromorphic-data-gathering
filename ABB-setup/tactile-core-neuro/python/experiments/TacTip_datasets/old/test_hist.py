# -*- coding: utf-8 -*-

import os, time, json, shutil
import pandas as pd

from core.model.hist.classification_hist import ClassificationHist
from test_CNN import plot_pred

# Specify directories and files
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip-datasets\edge5dTap')
test_meta_file = os.path.join(home_dir, 'collect_tap_rand_5d_11281815', 'meta.json')
model_meta_file = os.path.join(home_dir, 'collect_tap_rand_5d_11281815\models\train_hist_12180747', 'meta.json')


def make_test_meta(meta_file, test_meta_file, model_meta_file):
    test_dir =  os.path.dirname(test_meta_file)
    model_dir =  os.path.dirname(model_meta_file)
    os.makedirs(os.path.dirname(meta_file), exist_ok=True)
        
    # load metadata
    with open(model_meta_file, 'r') as f: meta = json.load(f)

    # update metadata
    test_pins_dir = os.path.join(test_dir, 'pins')
    test_df_file = os.path.join(test_dir, 'targets_pins.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
        
    # return metadata
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['test_meta_file'], meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta


def main(test_meta_file, model_meta_file):
    meta_file = os.path.join(os.path.dirname(test_meta_file), 'tests', 
        os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')
    meta = make_test_meta(meta_file, test_meta_file, model_meta_file)
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)

    temp_meta = {**meta, 'test_pins_dir': os.environ['TEMPPATH'] + r'\test'+time.strftime('%m%d%H%M')}           
    shutil.unpack_archive(meta['test_pins_dir']+r'.zip', temp_meta['test_pins_dir'])
    
    # startup/load model and make predictions on test data
    hist = ClassificationHist()
    hist.load_model(temp_meta['model_file'])
    pred = hist.predict_from_file(**temp_meta)
    hist.close()
    
    # analyze predictions
    pred_df = pd.read_csv(temp_meta['test_df_file'])
    for i, item in enumerate(temp_meta['target_names'], start=1):
        pred_df['pred_'+str(i)] = pred[:, i-1]
        pred_df['target_'+str(i)] = pred_df[item]
        pred_df['error_'+str(i)] = abs(pred_df['pred_'+str(i)] - pred_df['target_'+str(i)])
    pred_df.to_csv(os.path.dirname(meta_file) + r'\predictions.csv')
    fig = plot_pred(pred_df, **meta)
    fig.savefig(os.path.dirname(meta_file) + r'\errors.png', bbox_inches='tight')   
    
    shutil.rmtree(temp_meta['test_pins_dir'])

if __name__ == '__main__':
    main(test_meta_file, model_meta_file)

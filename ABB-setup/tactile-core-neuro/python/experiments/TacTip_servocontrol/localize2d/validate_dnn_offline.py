# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings, cv2
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
from core.model.dnn.CNN_model import CNNmodel
from experiments.TacTip_datasets.test_dnn import make_test_meta, plot_pred


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], r'TacTip_datasets\edge5dTap')
    test_meta_file = os.path.join(home_dir, r'collectTapRand5d06200640\meta.json')
    model_meta_file = os.path.join(home_dir, r'collectTapRand5d06200640\models\train_dnn_opt_01100403\meta_155.json')

    # Make test meta
    test_home_dir = os.path.join(os.environ['DATAPATH'], r'TacTip_servocontrol\validate')
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(test_home_dir, test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, test_meta_file, model_meta_file)
    
#    os.makedirs(os.path.dirname(meta_file), exist_ok=True) 
#    with open(meta_file, 'w') as f: 
#        json.dump(meta, f)   
    
    # unpack to temporary folder    
    temp_meta = {**meta, 'test_image_dir': os.environ['TEMPPATH'] + r'\test'+time.strftime('%m%d%H%M')}           
    shutil.unpack_archive(meta['test_image_dir']+r'.zip', temp_meta['test_image_dir'])
    
    # startup/load model and make predictions on test data
    cnn = CNNmodel()
    cnn.load_model(**temp_meta)
    cnn.print_model_summary()
    
    # make predictions looping through images
    test_df = pd.read_csv(temp_meta['test_df_file'])
    pred_df = pd.DataFrame()
    for i, row in test_df[::5].iterrows():
        image_file = os.path.join(temp_meta['test_image_dir'], row.loc['sensor_image'])
        image = cv2.imread(image_file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = image.astype('float32') / 255
        image = image[np.newaxis, ..., np.newaxis]
        
        pred = cnn.predict(image)
        target = np.array(row.loc[temp_meta['target_names']], dtype='float32')
        error = np.abs( target - pred )
        with np.printoptions(precision=2, suppress=True):
            print(f'{i+1}: pred={pred}, target={target}, error={error}')
    
        pred_row = row
        for j, jtem in enumerate(temp_meta['target_names']):
            pred_row['pred_'+str(j+1)] = pred[j]
            pred_row['target_'+str(j+1)] = target[j]
            pred_row['error_'+str(j+1)] = error[j]
        pred_df = pred_df.append(pred_row, ignore_index=True)
#    pred_df.to_csv(os.path.dirname(meta_file) +r'\pred.csv')
    
    fig = plot_pred(pred_df, **meta)
#    fig.savefig(os.path.dirname(meta_file) + r'\errors.png', bbox_inches='tight')   
    
    # tidy up
    shutil.rmtree(temp_meta['test_image_dir'])

if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
from core.model.dnn.CNN_model import CNNmodel
from experiments.TacTip_datasets.train_dnn import make_model_meta, plot_history


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge2dTap')
    train_meta_file = os.path.join(home_dir, 'collect_tap_rand_2d_02072153\meta.json')
    valid_meta_file = os.path.join(home_dir, 'collect_tap_rand_2d_02072153\meta.json')                                            
    
    # Make meta data
    model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
    meta = make_model_meta(meta_file, train_meta_file, valid_meta_file,
                            num_conv_layers = 5,                                                                   
                            num_conv_filters = 512,                                                                
                            num_dense_layers = 1,                                                                  
                            num_dense_units = 16,  
                            activation = 'elu',                                                         
                            dropout = 0.020431,                                                         
                            kernel_l1 = 0.000101,                                                       
                            kernel_l2 = 0.000149,                                                        
                            batch_size = 32, 
                            epochs = 200,
                            patience = 50,
                            loss_weights = [1/9**2, 1/45**2],
                            target_names = ['pose_1', 'pose_6'],
                            )
    
    # Save meta data
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)
     
    # unpack images to temporary folder
    temp_meta = {**meta, 'train_image_dir': os.environ['TEMPPATH'] + r'\train'+time.strftime('%m%d%H%M'),
                         'valid_image_dir': os.environ['TEMPPATH'] + r'\valid'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['train_image_dir']+r'.zip', temp_meta['train_image_dir'])
    shutil.unpack_archive(meta['valid_image_dir']+r'.zip', temp_meta['valid_image_dir'])
        
    # startup CNN, build and compile model
    cnn = CNNmodel()
    cnn.build_model(**temp_meta)
    cnn.compile_model(**temp_meta)
    cnn.print_model_summary()
    history = cnn.fit_model(**temp_meta, verbose=1)
        
    # Save training history (csv)
    history_df = pd.DataFrame(history)
    history_df.index += 1 
    history_df.to_csv(os.path.dirname(meta_file) + r'\history.csv', index=False)
    fig = plot_history(history_df)   
    fig.savefig(os.path.dirname(meta_file) + r'\history.png', bbox_inches='tight', pad_inches=0)
    
    # clean up
    shutil.rmtree(temp_meta['train_image_dir'])
    shutil.rmtree(temp_meta['valid_image_dir'])

if __name__ == '__main__':
    main()

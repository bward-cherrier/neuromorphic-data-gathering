# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.dnn.CNN_model import CNNmodel

def make_model_meta(meta_file, train_meta_file,
                    num_conv_layers = 5,                                                                   
                    num_conv_filters = 256,                                                                
                    num_dense_layers = 2,                                                                  
                    num_dense_units = 256,  
                    activation = 'relu',                                                         
                    dropout = 0.0334,                                                         
                    kernel_l1 = 0.00017,                                                       
                    kernel_l2 = 0.00065,                                                        
                    batch_size = 16, 
                    epochs = 200,
                    patience = 10,
                    lr = 1e-4,
                    decay = 1e-6,
                    loss_weights = [1/5**2, 1/4**2, 1/15**2, 1/15**2, 1/45**2],
                    target_names = ['pose_1', 'pose_3', 'pose_4', 'pose_5', 'pose_6'],
                    ):
    model_dir = os.path.dirname(meta_file)
    train_dir = os.path.dirname(train_meta_file)

    # load metadata
    with open(train_meta_file, 'r') as f: 
        meta = json.load(f)    
        
    # update metadata
    model_file = os.path.join(model_dir, 'model.h5')
    train_image_dir = os.path.join(train_dir, 'images_bw')
    train_df_file = os.path.join(train_dir, 'targets_image.csv')
    valid_split = 0.2      
    size = [165, 165]    

    # save metadata in model dir
    meta.update(locals().copy())
    del meta['train_meta_file'] 
    del meta['image_dir'], meta['image_df_file'] 
    del meta['meta'], meta['f'], meta['train_dir'], meta['model_dir']
    try: del meta['pins_dir'], meta['pins_df_file'], meta['pins_init_file'] 
    except: pass
    return meta

def plot_history(history_df):
    plt.rcParams.update({'font.size': 18})
    fig, ax = plt.subplots(ncols = 1, figsize=(7, 7))
    history_df.plot(ax=ax, y='loss', label='Training loss')
    history_df.plot(ax=ax, y='val_loss', label='Validation loss')
    ax.set(xlabel='epochs', ylabel='loss')
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.legend()
    min_loss = (min(history_df.loss), min(history_df.val_loss))
    fig.suptitle('Training loss: %3.2f Validation loss: %3.2f' % min_loss)    
    return fig
   

def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
    train_meta_file = os.path.join('edge5d', 'collect5d_rand_02271105', 'meta.json')

    # Make meta data
    model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
    meta = make_model_meta(home_dir, meta_file, train_meta_file)
    
    # Save meta data
    os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))
    with open(os.path.join(home_dir, meta_file), 'w') as f: 
        json.dump(meta, f)
     
    # unpack images to temporary folder
    temp_dir = os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))
    shutil.unpack_archive(os.path.join(home_dir, meta['train_image_dir']+r'.zip'), os.path.join(temp_dir, meta['train_image_dir']))

    temp_meta = {**meta, 'train_image_dir': os.environ['TEMPPATH'] + r'\train'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['train_image_dir']+r'.zip', temp_meta['train_image_dir'])
        
    # startup CNN, build and compile model
    cnn = CNNmodel()
    cnn.build_model(**temp_meta)
    cnn.compile_model(**temp_meta)
    cnn.print_model_summary()
    history = cnn.fit_model(**temp_meta, verbose=1)
        
    # Save training history (csv)
    history_df = pd.DataFrame(history)
    history_df.index += 1 
    history_df.to_csv(os.path.join(home_dir, os.path.dirname(meta_file), 'history.csv'), index=False)
    fig = plot_history(history_df)   
    fig.savefig(os.path.join(home_dir,os.path.dirname(meta_file), 'history.png'), bbox_inches='tight', pad_inches=0)
    
    # clean up
    shutil.rmtree(temp_meta['train_image_dir'])

if __name__ == '__main__':
    main()

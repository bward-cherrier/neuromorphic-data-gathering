# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.dnn.CNN_model import CNNmodel

def make_model_meta(home_dir, meta_file, train_meta_file, valid_meta_file, 
                    num_conv_layers = 5, 
                    num_conv_filters = 256, 
                    num_dense_layers = 1, 
                    num_dense_units = 256, 
                    activation = "relu", 
                    dropout = 0.10608355360378807, 
                    kernel_l1 = 0.00015388797796823684, 
                    kernel_l2 = 0.00019677541040839908,                                                        
                    batch_size = 16, 
                    epochs = 200,
                    patience = 10,
                    lr = 1e-4,
                    decay = 1e-6,
                    loss_weights = [1/25, 1/10, 1/250, 1/250, 1/2000],
                    target_names = ['pose_1', 'pose_3', 'pose_4', 'pose_5', 'pose_6'],
                    ):
    model_dir = os.path.dirname(meta_file)
    model_file = os.path.join(model_dir, 'model.h5')
    size = [128, 128]

    # load valid metadata
    with open(os.path.join(home_dir, valid_meta_file), 'r') as f: 
        meta = json.load(f)  
    valid_image_dir = meta['image_dir']         
    valid_df_file = meta['image_df_file']

    # load train metadata
    with open(os.path.join(home_dir, train_meta_file), 'r') as f: 
        meta = json.load(f)    
    train_image_dir = meta['image_dir']         
    train_df_file = meta['image_df_file']

    # return metadata
    meta.update(locals().copy())
    del meta['train_meta_file'], meta['valid_meta_file'], meta['model_dir'] 
    del meta['meta'], meta['f'], meta['image_dir'], meta['image_df_file'] 
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


#def main():
# Specify directories and files - to edit
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
train_meta_file = os.path.join('edge5d', 'collect5d_rand_03091020', 'version4', 'meta.json')
valid_meta_file = os.path.join('edge5d', 'collect5d_rand_03091459', 'version4', 'meta.json')                                            

# Make meta data
model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
meta = make_model_meta(home_dir, meta_file, train_meta_file, valid_meta_file)

# Save meta data
os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))
with open(os.path.join(home_dir, meta_file), 'w') as f: 
    json.dump(meta, f)
 
# unpack images to temporary folder
temp_dir = os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))
shutil.unpack_archive(os.path.join(home_dir, meta['train_image_dir']+r'.zip'), os.path.join(temp_dir, meta['train_image_dir']))
shutil.unpack_archive(os.path.join(home_dir, meta['valid_image_dir']+r'.zip'), os.path.join(temp_dir, meta['valid_image_dir']))

# full paths for functions
meta['train_image_dir'] = os.path.join(temp_dir, meta['train_image_dir'])
meta['valid_image_dir'] = os.path.join(temp_dir, meta['valid_image_dir'])
meta['train_df_file'] = os.path.join(home_dir, meta['train_df_file'])
meta['valid_df_file'] = os.path.join(home_dir, meta['valid_df_file'])
meta['model_file'] = os.path.join(home_dir, meta['model_file'])
     
# startup CNN, build and compile model
cnn = CNNmodel()
cnn.build_model(**meta)
cnn.compile_model(**meta)
cnn.print_model_summary()
history = cnn.fit_model(**meta, verbose=1)
    
# Save training history (csv)
history_df = pd.DataFrame(history)
history_df.index += 1 
history_df.to_csv(os.path.join(home_dir, os.path.dirname(meta_file), 'history.csv'), index=False)
fig = plot_history(history_df)   
fig.savefig(os.path.join(home_dir, os.path.dirname(meta_file), 'history.png'), bbox_inches='tight', pad_inches=0)

# clean up
shutil.rmtree(meta['train_image_dir'])
shutil.rmtree(meta['valid_image_dir'])

#if __name__ == '__main__':
#    main()

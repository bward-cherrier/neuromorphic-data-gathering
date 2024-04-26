# -*- coding: utf-8 -*-

import os, time, json, shutil, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import matplotlib.pyplot as plt
from core.model.cnn.regression_cnn import RegressionCNN

# Specify directories and files - to edit
home_dir = os.path.join(os.environ['DATAPATH'], r'TacTip-datasets\edge5dTap - copy')
train_meta_file = os.path.join(home_dir, 'collectTapRand5d06200640\meta.json')
valid_meta_file = os.path.join(home_dir, 'collectTapRand5d06200641\meta.json')


def make_model_meta(meta_file, train_meta_file, valid_meta_file,
                    crop=(160, 70, 490, 400),
                    size=(128, 128),
                    threshold=(11, -5),
                    conv_filters=(512,)*4,
                    dense_units=(256,)*1,
                    activation='relu',
                    dropout=(0,)*4 + (0.013019,)*2,
                    batch_norm=(False,)*4 + (True,)*2,
                    kernel_l1=(0,)*4 + (0.000128,)*2,
                    kernel_l2=(0,)*4 + (0.000445,)*2,
                    batch_size=32,
                    epochs=20,
                    patience=10,
                    valid_split=0,
                    lr=1e-4,
                    decay=1e-6,
                    loss_weights=[1/5**2, 1/3**2, 1/15**2, 1/15**2, 1/45**2],
                    target_names=['pose_1', 'pose_3', 'pose_4', 'pose_5', 'pose_6'],
                    ):
    model_dir = os.path.dirname(meta_file)
    train_dir = os.path.dirname(train_meta_file)
    valid_dir = os.path.dirname(valid_meta_file)

    # load metadata
    with open(train_meta_file, 'r') as f:
        meta = json.load(f)

    # update metadata
    model_file = os.path.join(model_dir, 'model.h5')
    train_image_dir = os.path.join(train_dir, 'images')
    train_df_file = os.path.join(train_dir, 'targets_image.csv')
    valid_image_dir = os.path.join(valid_dir, 'images')
    valid_df_file = os.path.join(valid_dir, 'targets_image.csv')

    # return metadata
    meta.update(locals().copy())
    del meta['train_meta_file'], meta['valid_meta_file']
    del meta['video_dir'], meta['video_df_file'], meta['image_dir'], meta['image_df_file']
    del meta['meta'], meta['f'], meta['train_dir'], meta['valid_dir'], meta['model_dir']
    try:
        del meta['pins_dir'], meta['pins_df_file'], meta['pins_init_file']
    except:
        pass
    return meta

def plot_history(history_df):
    plt.rcParams.update({'font.size': 18})
    fig, ax = plt.subplots(ncols=1, figsize=(7, 7))
    history_df.plot(ax=ax, y='loss', label='Training loss')
    history_df.plot(ax=ax, y='val_loss', label='Validation loss')
    ax.set(xlabel='epochs', ylabel='loss')
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.legend()
    min_loss = (min(history_df.loss), min(history_df.val_loss))
    fig.suptitle('Training loss: %3.2f Validation loss: %3.2f' % min_loss)
    return fig


def main(train_meta_file, valid_meta_file):
    model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
    meta = make_model_meta(meta_file, train_meta_file, valid_meta_file)
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)
    
    # unpack to temporary folder
    temp_meta = {**meta, 'train_image_dir': os.path.join(os.environ['TEMPPATH'], 'train' + time.strftime('%m%d%H%M')),
                 'valid_image_dir': os.path.join(os.environ['TEMPPATH'], 'valid' + time.strftime('%m%d%H%M'))}
    shutil.unpack_archive(meta['train_image_dir'] + r'.zip', temp_meta['train_image_dir'])
    shutil.unpack_archive(meta['valid_image_dir'] + r'.zip', temp_meta['valid_image_dir'])
    
    # startup CNN, build and compile model
    cnn = RegressionCNN()
    cnn.build_model(**temp_meta)
    cnn.compile_model(**temp_meta)
    cnn.print_model_summary()
    history = cnn.fit_from_file(**temp_meta)
    
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
    main(train_meta_file, valid_meta_file)

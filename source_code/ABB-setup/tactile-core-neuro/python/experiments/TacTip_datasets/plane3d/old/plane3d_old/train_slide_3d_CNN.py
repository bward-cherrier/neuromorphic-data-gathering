# -*- coding: utf-8 -*-
"""
Created on Fri Feb  1 09:58:05 2019

@author: nl13426
"""

import os, time, json, shutil
import pandas as pd
import matplotlib.pyplot as plt
from core.model.cnn.regression_cnn import RegressionCNN

# Specify directories and files
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip-datasets', 'plane3d')
data_dir_name = 'collectSlideRand3dNframes08161549'
#valid_dir_name = 'collectSlideRand3dNframes08161549'
model_dir_name = r'models\train_slide_3d_CNN_' + time.strftime("%m%d%H%M") 

#______________________________________________________________________________
# Specify training parameters

data_dir = os.path.join(home_dir, data_dir_name)
#valid_dir = os.path.join(home_dir, valid_dir_name)
model_dir = os.path.join(data_dir, model_dir_name)

with open(os.path.join(data_dir, 'meta.json'), 'r') as f:
    meta_dict = json.load(f)

build_dict = {
        'input_dims': (128, 128, 1),
        'output_dim': 3,
        'conv_filters': (256, 256, 256, 256, 256),
        'dense_units': (256, ),
        'activation': 'elu',
        'batch_norm': (False, False, False, False, False, False, False),
        'dropout': (0.0, 0.0, 0.0, 0.0, 0.0, 0.01867, 0.01867),
        'kernel_reg_l1': (0.0, 0.0, 0.0, 0.0, 0.0, 0.001042, 0.001042),
        'kernel_reg_l2': (0.0, 0.0, 0.0, 0.0, 0.0, 0.08929, 0.08929),
        }

compile_dict = {
        'learning_rate': 1e-4, 
        'epsilon': 1e-8,
        'learning_rate_decay': 1e-6,
        }

fit_dict = {
        'image_dir': os.path.join(data_dir, 'images'), 
        'image_target_file': os.path.join(data_dir, 'targets_image.csv'), 
#        'validation_image_dir': os.path.join(valid_dir, 'images'), 
#        'validation_target_file': os.path.join(valid_dir, 'targets_image.csv'),         
        'target_names': ['pose_3', 'pose_4', 'pose_5'],
        'batch_size': 32,
        'epochs': 200,
        'validation_split': 0.2,
        'patience': 7,
        'bbox': (160, 70, 490, 400),
        'dims': tuple(build_dict['input_dims'][:-1]),
        'threshold': (11, -5),
        'best_model_file': os.path.join(model_dir, 'best_model.h5'),
        }

# Append to collect meta data (json)
os.makedirs(model_dir)
meta_dict.update(dict(**build_dict, **compile_dict, **fit_dict))
with open(os.path.join(model_dir, 'meta.json'), 'w') as f:
    json.dump(meta_dict, f)

# Extract images to temp directory
image_dir = os.path.join('D:\Temp', data_dir_name+'_images')
shutil.unpack_archive(fit_dict['image_dir']+r'.zip', image_dir)
fit_dict['image_dir'] = image_dir

#image_dir = os.path.join('D:\Temp', valid_dir_name+'_images')
#shutil.unpack_archive(fit_dict['validation_image_dir']+r'.zip', image_dir)
#fit_dict['validation_image_dir'] = image_dir

#______________________________________________________________________________       
# Startup CNN, build and compile model
          
cnn = RegressionCNN()
cnn.build_model(**build_dict)
cnn.compile_model(**compile_dict)
cnn.print_model_summary()
history = cnn.fit_from_file(**fit_dict)

# Save training history (csv)
history_df = pd.DataFrame(history)
history_df.index += 1 
history_df.to_csv(os.path.join(model_dir, 'history.csv'), index=False)

#______________________________________________________________________________     
# Plot training history
    
fig = plt.figure(figsize=(3.5, 3.5))
ax = history_df.plot(y='loss', label='Training loss')
history_df.plot(ax=ax, y ='val_loss', label='Validation loss')
ax.set(xlabel='epochs', ylabel='loss')
ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
ax.legend()
min_loss = (min(history_df.loss), min(history_df.val_loss))
plt.title('Training loss: %3.2f Validation loss: %3.2f' % min_loss)

plt.savefig(os.path.join(model_dir, 'history.png'), bbox_inches='tight', pad_inches=0)

# Tidy up
shutil.rmtree(fit_dict['image_dir'])
shutil.rmtree(fit_dict['validation_image_dir'])

#!/usr/bin/env python
# coding: utf-8

# # CNN hyperparameter optimization

# In[1]:


import os
import math
import datetime
import pickle
import json
import copy
from functools import partial

import cv2
import numpy as np
import pandas as pd

from hyperopt import tpe, hp, fmin, Trials, STATUS_OK, STATUS_FAIL
from hyperopt.pyll import scope

import tensorflow as tf

import keras
from keras import layers, models, optimizers, regularizers, callbacks
from keras import backend as K
from keras_preprocessing.image import ImageDataGenerator


# In[2]:


# hyperopt helper function
@scope.define
def exp2(x):
    return 2 ** x


# In[3]:


# decorator for multi-output layer generators
class MultiOutputGenerator:
    def __init__(self, gen):
        self._gen = gen
    
    def __iter__(self):
        return self
    
    def __next__(self):
        ret = self._gen.next()
        x, y = ret
        y = [y_col for y_col in y.T] if y.ndim > 1 else [y]
        return x, y
    
    @property
    def n(self):
        return self._gen.n
    
    @property
    def batch_size(self):
        return self._gen.batch_size


# In[4]:


# configure Tensorflow GPU options
config = tf.ConfigProto(
    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.8)
    # device_count = {'GPU': 1}
)
config.gpu_options.allow_growth = True
session = tf.Session(config=config)
K.set_session(session)


# In[5]:


# set up data dirs and files
data_root_dir = os.environ["DATAPATH"]
#data_root_dir = r"D:\Users\nl13426\Documents\Data temp"

train_data_dir = os.path.join(data_root_dir, r"TacTip-datasets\plane3d\collectSlideRand3dNframes05071937")
train_image_dir = os.path.join(train_data_dir, "images")
train_df_file = os.path.join(train_data_dir, "targets_image.csv")
train_df = pd.read_csv(train_df_file)

valid_data_dir = os.path.join(data_root_dir, r"TacTip-datasets\plane3d\collectSlideRand3dNframes05071938")
valid_image_dir = os.path.join(valid_data_dir, "images")
valid_df_file = os.path.join(valid_data_dir, "targets_image.csv")
valid_df = pd.read_csv(valid_df_file)


# In[6]:


# function for pre-processing images
def process_image(image, crop=None, resize=None, threshold=None): 
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if crop is not None:
        x0, y0, x1, y1 = crop
        image = image[y0:y1, x0:x1]
    if threshold is not None:
#        ret, image = cv2.threshold(image, int(threshold * 255), 255, cv2.THRESH_BINARY)
        image = image.astype('uint8')
        image = cv2.medianBlur(image, 5)
        width, offset = threshold
        image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,                                       cv2.THRESH_BINARY, width, offset)
    if resize is not None:
        image = cv2.resize(image, resize, interpolation=cv2.INTER_AREA)
    image = image[..., np.newaxis]  
    return image


# In[7]:


crop = (160, 70, 490, 400)
resize = (128, 128)
threshold = (11, -5)

# pre-process training images
pp_train_image_dir = os.path.join(train_image_dir, 'pp_images')
if not os.path.isdir(pp_train_image_dir):
    os.makedirs(pp_train_image_dir)
    image_files = (f for f in os.listdir(train_image_dir) if f.endswith('.jpg'))
    for image_file in image_files:
        image = cv2.imread(os.path.join(train_image_dir, image_file))
        image = process_image(image, crop=crop, resize=resize, threshold=threshold)
        cv2.imwrite(os.path.join(pp_train_image_dir, image_file), image);
        
# pre-process validation images
pp_valid_image_dir = os.path.join(valid_image_dir, 'pp_images')
if not os.path.isdir(pp_valid_image_dir):
    os.makedirs(pp_valid_image_dir)
    image_files = (f for f in os.listdir(valid_image_dir) if f.endswith('.jpg'))
    for image_file in image_files:
        image = cv2.imread(os.path.join(valid_image_dir, image_file))
        image = process_image(image, crop=crop, resize=resize, threshold=threshold)
        cv2.imwrite(os.path.join(pp_valid_image_dir, image_file), image);


# In[8]:


# build CNN model
def build_model(num_conv_layers,
                num_conv_filters,
                num_dense_layers,
                num_dense_units,
                hidden_activation,
                batch_norm,
                dropout,
                l1_norm,
                l2_norm,
                batch_size,
                ):
    
    input_dims = (128, 128, 1)
    output_dim = 3
    num_hidden_layers = num_conv_layers + num_dense_layers
    loss_weights = [5**-2, 15**-2, 15**-2] 

    # convolutional layers
    inp = keras.Input(shape=input_dims, name='input')
    x = inp
    for i in range(num_conv_layers):
        x = layers.Conv2D(filters=num_conv_filters, kernel_size=3)(x)
        if batch_norm:
            x = layers.BatchNormalization()(x)
        x = layers.Activation(hidden_activation)(x)
        x = layers.MaxPooling2D()(x)

    # dense layers
    x = layers.Flatten()(x)
    for i in range(num_conv_layers, num_hidden_layers):
        if dropout > 0:
            x = layers.Dropout(dropout)(x)
        x = layers.Dense(units=num_dense_units,
            kernel_regularizer=regularizers.l1_l2(l1=l1_norm, l2=l2_norm))(x)         
        x = layers.Activation(hidden_activation)(x)            

    # Output layers
    if dropout > 0:
        x = layers.Dropout(dropout)(x)
    out = [layers.Dense(units=1,
                name=('output_' + str(i + 1)),
                kernel_regularizer=regularizers.l1_l2(l1=l1_norm, l2=l2_norm))(x)
                for i in range(output_dim)]

    model = models.Model(inp, out)
    model.compile(optimizer=optimizers.Adam(lr=1e-4, decay=1e-6),
                                            loss=['mse',] * output_dim,
                                            loss_weights=loss_weights,
                                            metrics=['mae']) 
    return model


# In[9]:


# build hyperopt objective function
def build_objective_func():
    trial = 1
    
    def objective_func(args):
        nonlocal trial
        global model_dir

        print("Trial: {}".format(trial))
        
        # unpack params
        num_conv_layers = args['num_conv_layers']
        num_conv_filters = args['num_conv_filters']
        num_dense_layers = args['num_dense_layers']
        num_dense_units = args['num_dense_units']
        hidden_activation = args['hidden_activation']
        batch_norm = args['batch_norm']
        dropout = args['dropout']
        l1_norm = args['l1_norm']
        l2_norm = args['l2_norm']
        batch_size = args['batch_size']

        # print params
        print("num_conv_layers: {}".format(num_conv_layers))
        print("num_conv_filters: {}".format(num_conv_filters))
        print("num_dense_layers: {}".format(num_dense_layers))
        print("num_dense_units: {}".format(num_dense_units))
        print("hidden_activation: {}".format(hidden_activation))
        print("batch_norm: {}".format(batch_norm))
        print("dropout: {:0.2}".format(dropout))
        print("l1_norm: {:0.2}".format(l1_norm))
        print("l2_norm: {:0.2}".format(l2_norm))
        print("batch_size: {}".format(batch_size))

        # build CNN model
        K.clear_session()
        model = build_model(num_conv_layers,
                            num_conv_filters,
                            num_dense_layers,
                            num_dense_units,
                            hidden_activation,
                            batch_norm,
                            dropout,
                            l1_norm,
                            l2_norm,
                            batch_size,
                           )
        num_params = model.count_params()
        print("Model params: {:,}".format(num_params))

        # other training parameters
        x_col = 'sensor_image'
        y_col = ['pose_3', 'pose_4', 'pose_5']
        class_mode='other'
        target_size = (128, 128)
        color_mode = 'grayscale'
        patience = 10
        epochs = 3

        # set up data generators
#        train_datagen = ImageDataGenerator(rescale=1./255., validation_split=0.2)
        train_datagen = ImageDataGenerator(rescale=1./255.)
        valid_datagen = ImageDataGenerator(rescale=1./255.)
        
        train_generator = train_datagen.flow_from_dataframe(
            dataframe=train_df,
            directory=pp_train_image_dir,
            x_col=x_col,
            y_col=y_col,
#            subset='training',
            batch_size=batch_size,
            seed=42,
            shuffle=True,
            class_mode=class_mode,
            target_size=target_size,
            color_mode=color_mode,
        )

        valid_generator = valid_datagen.flow_from_dataframe(
            dataframe=valid_df,
            directory=pp_valid_image_dir,
            x_col=x_col,
            y_col=y_col,
#            subset='validation',
            batch_size=batch_size,
            seed=42,
            shuffle=True,
            class_mode=class_mode,
            target_size=target_size,
            color_mode=color_mode,
        )
        
        train_generator = MultiOutputGenerator(train_generator)
        valid_generator = MultiOutputGenerator(valid_generator)    
        step_size_train = train_generator.n // train_generator.batch_size
        step_size_valid = valid_generator.n // valid_generator.batch_size

        # fit model
        print("Training model ...")
        model_file = "best_model_" + str(trial) + ".h5"
        train_callbacks = [
            callbacks.EarlyStopping(monitor='val_loss', patience=patience, verbose=1),
            callbacks.ModelCheckpoint(
                filepath=os.path.join(model_dir, model_file),
                monitor='val_loss',
                save_best_only=True,
            )
        ]
        try:
            history = model.fit_generator(generator=train_generator,
                                          steps_per_epoch=step_size_train,
                                          validation_data=valid_generator,
                                          validation_steps=step_size_valid,
                                          epochs=epochs,
                                          verbose=0,
                                          callbacks=train_callbacks)
        except tf.errors.ResourceExhaustedError:
            loss = None
            status = STATUS_FAIL
            stopped_epoch = None
            history = None
            print("Aborted trial: Resource exhausted error\n")
        else:
            loss = np.min(history.history['val_loss'])
            status = STATUS_OK
            stopped_epoch = len(history.history['val_loss'])
            history = history.history
            print("Loss: {:0.2}\n".format(loss)) 

        results = {'loss': loss, 'status': status, 'stopped_epoch': stopped_epoch,
                   'history': history, 'num_params': num_params, 'trial': trial,}
        trial += 1
        return results  
    
    return objective_func


# In[10]:


# specify hyperopt search space
# num_conv_layers = scope.int(hp.quniform(label='num_conv_layers', low=1, high=5, q=1))
space = {
#    'num_conv_layers': num_conv_layers,
#    'num_conv_filters': scope.int(scope.exp2(scope.switch(num_conv_layers - 1,
#        hp.quniform(label='num_conv_filters_1', low=1, high=8, q=1), 
#        hp.quniform(label='num_conv_filters_2', low=1, high=7, q=1),
#        hp.quniform(label='num_conv_filters_3', low=1, high=6, q=1),
#        hp.quniform(label='num_conv_filters_4', low=1, high=5, q=1),
#        hp.quniform(label='num_conv_filters_5', low=1, high=4, q=1),
#    ))),
    'num_conv_layers': scope.int(hp.quniform(label='num_conv_layers', low=1, high=5, q=1)),
    'num_conv_filters': scope.int(scope.exp2(hp.quniform(label='num_conv_filters', low=1, high=9, q=1))),
    'num_dense_layers': scope.int(hp.quniform(label='num_dense_layers', low=1, high=5, q=1)),
    'num_dense_units': scope.int(scope.exp2(hp.quniform(label='num_dense_units', low=1, high=9, q=1))),
    'hidden_activation':  hp.choice(label='hidden_activation', options=('relu', 'elu')),
    'batch_norm':  hp.choice(label='batch_norm', options=(False, True)),
    'dropout': hp.uniform(label='dropout', low=0, high=0.5),
    'l1_norm': hp.loguniform(label='l1_norm', low=-4*math.log(10), high=-math.log(10)),
    'l2_norm': hp.loguniform(label='l2_norm', low=-4*math.log(10), high=-math.log(10)),
    'batch_size': scope.int(scope.exp2(hp.quniform(label='batch_size', low=4, high=7, q=1))),
}


# In[11]:


# helper function for formatting returned hyperparams
def format_params(params):
    params_conv = copy.deepcopy(params)
    params_conv['batch_norm'] = True if params['batch_norm'] == 1 else False
    params_conv['batch_size'] = int(2 ** params['batch_size'])
    params_conv['hidden_activation'] = ('relu', 'elu')[params['hidden_activation']]
#    num_conv_layers = int(params['num_conv_layers'])
#    params_conv['num_conv_layers'] = num_conv_layers
#    del_keys = [k for k in params_conv.keys() if k.lower().startswith('num_conv_filters_')]
#    for k in del_keys:
#        del params_conv[k]
#    params_conv['num_conv_filters'] = int(2 ** params['num_conv_filters_' + str(num_conv_layers)])
    params_conv['num_conv_layers'] = int(params['num_conv_layers'])
    params_conv['num_conv_filters'] = int(2 ** params['num_conv_filters'])
    params_conv['num_dense_layers'] = int(params['num_dense_layers'])
    params_conv['num_dense_units'] = int(2 ** params['num_dense_units'])
    
    return params_conv


# In[13]:


# set up model dir
model_dir = os.path.join(train_data_dir, "models", "opt_" + datetime.datetime.now().strftime("%m%d%H%M"))
if not os.path.isdir(model_dir):
    os.makedirs(model_dir)

# perform optimization
trials = Trials()
obj_func = build_objective_func()
best_model_params = fmin(obj_func, space, algo=partial(tpe.suggest, n_startup_jobs=50), max_evals=300, trials=trials)

# save (formatted) best model params
with open(os.path.join(model_dir, "best_model_params.json"), "w") as f:
    json.dump(format_params(best_model_params), f)
    
# save trials history
with open(os.path.join(model_dir, "trials.pickle"), "wb") as f:
    pickle.dump(trials, f)

# save results in CSV format
trials_df = pd.DataFrame()
for i, trial in enumerate(trials):
    trial_params = {k: v[0] if len(v) > 0 else None for k, v in trial['misc']['vals'].items()}
    trial_row = pd.DataFrame(format_params(trial_params), index=[i])
    trial_row['tid'] = trial['tid']
    trial_row['loss'] = trial['result']['loss']
    trial_row['status'] = trial['result']['status']
    trial_row['trial'] = trial['result']['trial']
    trial_row['stopped_epoch'] = trial['result']['stopped_epoch']
    trial_row['num_params'] = trial['result']['num_params']
    trials_df = pd.concat([trials_df, trial_row])
# trials_df['loss'] = trials.losses()
trials_df.to_csv(os.path.join(model_dir, "results.csv"), index=False)


# In[14]:


# print results
losses = copy.deepcopy(trials.losses())
losses = [l if l is not None else np.inf for l in losses]

print("Best model trial: {}".format(np.argmin(losses) + 1))
print("Best model loss: {:0.4}".format(np.min(losses)))
print("Best model params:")
print(format_params(best_model_params))


# In[15]:


# load and print trials history
with open(os.path.join(model_dir, "trials.pickle"), "rb") as f:
    trials = pickle.load(f)
    
for t in trials.trials:
    print(t)


# In[ ]:





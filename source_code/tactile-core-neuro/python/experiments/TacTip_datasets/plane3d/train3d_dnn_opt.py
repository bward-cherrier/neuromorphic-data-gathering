# coding: utf-8

import os, math, time, json, shutil, copy, pickle, warnings
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from functools import partial
from hyperopt.pyll import scope
import tensorflow as tf
from hyperopt import tpe, hp, fmin, Trials, STATUS_OK, STATUS_FAIL
from core.model.dnn.CNN_model import CNNmodel

def make_model_meta(home_dir, meta_file, train_meta_file, valid_meta_file,
                    num_conv_layers = 5, 
                    num_conv_filters = 256, 
                    num_dense_layers = 1, 
                    num_dense_units = 256, 
                    activation = None,
                    dropout = None,
                    kernel_l1 = None,
                    kernel_l2 = None,
                    batch_size = 16,
                    epochs = 200,
                    patience = 10,
                    lr = 1e-4,
                    decay = 1e-6,
                    loss_weights = [1/25, 1/250, 1/250],
                    target_names = ['pose_3', 'pose_4', 'pose_5'],
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
    del meta['train_meta_file'], meta['valid_meta_file']
    del meta['image_dir'], meta['image_df_file'] 
    del meta['meta'], meta['f'], meta['model_dir']
    try: del meta['pins_dir'], meta['pins_df_file'], meta['pins_init_file'] 
    except: pass
    return meta

# build hyperopt objective function
def build_objective_func(meta_0):
    trial = 1
    model_dir = os.path.dirname(meta_0['model_file'])
    
    def objective_func(args):
        nonlocal trial
        print("Trial: {}".format(trial))
        for x in args: print (x + ': ' + str(args[x]))
        
        # build and save metadata
        meta = {**meta_0, **args,
                'meta_file': os.path.join(model_dir, 'meta_'+str(trial)+'.json'),
                'model_file': os.path.join(model_dir, 'model_'+str(trial)+'.h5')}
        meta_save = meta.copy()
        del meta_save['temp_train_image_dir'], meta_save['temp_valid_image_dir']
        with open(os.path.join(meta['home_dir'], meta['meta_file']), 'w') as f: 
            json.dump(meta_save, f)     
        
        # use temporary directories
        meta['train_image_dir'] = meta['temp_train_image_dir']
        meta['valid_image_dir'] = meta['temp_valid_image_dir']
        meta['train_df_file'] = os.path.join(meta['home_dir'], meta['train_df_file'])
        meta['valid_df_file'] = os.path.join(meta['home_dir'], meta['valid_df_file'])
        meta['model_file'] = os.path.join(meta['home_dir'], meta['model_file'])

        # startup CNN, build and compile model
        cnn = CNNmodel()
        cnn.build_model(**meta)
        cnn.compile_model(**meta)
        try:
            history = cnn.fit_model(**meta)
        except tf.errors.ResourceExhaustedError:
            results = {'loss': None, 
                       'status': STATUS_FAIL, 
                       'stopped_epoch': None, 
                       'history': None}
            print("Aborted trial: Resource exhausted error\n")
        else:
            results = {'loss': np.min(history['val_loss']), 
                       'status': STATUS_OK, 
                       'stopped_epoch': len(history['val_loss']),
                       'history': history}
            print("Loss: {:0.2}\n".format(results['loss'])) 
        results = {**results, 'num_params': cnn._model.count_params(), 'trial': trial}
        trial += 1
        return results    
    return objective_func
    
def make_trials_df(trials):
    trials_df = pd.DataFrame()
    for i, trial in enumerate(trials):
        trial_params = {k: v[0] if len(v) > 0 else None for k, v in trial['misc']['vals'].items()}
        trial_row = pd.DataFrame(format_params(trial_params), index=[i])
        trial_row['loss'] = trial['result']['loss']
        trial_row['tid'] = trial['tid']
        trial_row['status'] = trial['result']['status']
        trial_row['trial'] = trial['result']['trial']
        trial_row['stopped_epoch'] = trial['result']['stopped_epoch']
        trial_row['num_params'] = trial['result']['num_params']
        trials_df = pd.concat([trials_df, trial_row])
    return trials_df

def format_params(params):
    params_conv = copy.deepcopy(params)
#    params_conv['num_conv_layers'] = int(params['num_conv_layers'])
#    params_conv['num_conv_filters'] = int(2 ** params['num_conv_filters'])
#    params_conv['num_dense_layers'] = int(params['num_dense_layers'])
#    params_conv['num_dense_units'] = int(2 ** params['num_dense_units'])
    params_conv['activation'] = (0, 1)[params['activation']]
    return params_conv
 
def plot_trials(trials_df):
    trials_df.fillna(value=pd.np.nan, inplace=True)
    plt.rcParams.update({'font.size': 18})
    fig, axes = plt.subplots(nrows=5, ncols=2, figsize=(7*2, 7*5/2))
    fig.subplots_adjust(wspace=0.3)
    for i in range(0, 10): 
        trials_df.plot(ax=axes[i%5,i%2], x='trial', y=trials_df.keys()[i], 
                       s=2, c='red', kind='scatter', grid=True)    
        axes[i%5,i%2].set(ylabel=trials_df.keys()[i])
        axes[i%5,i%2].set_ylim(bottom=0)
    return fig   

#def main():
    # Specify directories and files - to edit
home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets')
train_meta_file = os.path.join('plane3d', 'collect3d_rand_03192000', 'version1', 'meta.json')
valid_meta_file = os.path.join('plane3d', 'collect3d_rand_03192000', 'version2', 'meta.json')                                         

# hyperopt search parameters
space = {
#    'num_conv_layers': scope.int(hp.quniform(label='num_conv_layers', low=1, high=5, q=1)),
#    'num_conv_filters': scope.int(2 ** (hp.quniform(label='num_conv_filters', low=1, high=9, q=1))),
#    'num_dense_layers': scope.int(hp.quniform(label='num_dense_layers', low=1, high=5, q=1)),
#    'num_dense_units': scope.int(2 ** (hp.quniform(label='num_dense_units', low=1, high=9, q=1))),
    'activation':  hp.choice(label='activation', options=('relu', 'elu')),
    'dropout': hp.uniform(label='dropout', low=0, high=0.5),
    'kernel_l1': hp.loguniform(label='kernel_l1', low=-4*math.log(10), high=-math.log(10)),
    'kernel_l2': hp.loguniform(label='kernel_l2', low=-4*math.log(10), high=-math.log(10))}
max_evals = 50
n_startup_jobs = 20

# make meta data
model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
meta = make_model_meta(home_dir, meta_file, train_meta_file, valid_meta_file)
os.makedirs(os.path.join(home_dir, os.path.dirname(meta_file)))

# unpack images to temporary folder
meta['temp_train_image_dir'] = os.path.join(os.environ['TEMPPATH'], 'train'+time.strftime('%m%d%H%M'))
meta['temp_valid_image_dir'] = os.path.join(os.environ['TEMPPATH'], 'valid'+time.strftime('%m%d%H%M'))
shutil.unpack_archive(os.path.join(home_dir, meta['train_image_dir']+r'.zip'), meta['temp_train_image_dir'])
shutil.unpack_archive(os.path.join(home_dir, meta['valid_image_dir']+r'.zip'), meta['temp_valid_image_dir'])
        
# perform optimization
trials = Trials()
obj_func = build_objective_func(meta)
opt_params = fmin(obj_func, space, max_evals=max_evals, trials=trials, 
                  algo=partial(tpe.suggest, n_startup_jobs=n_startup_jobs))
with open(os.path.join(home_dir, os.path.dirname(meta_file), "trials.pickle"), "wb") as f:
    pickle.dump(trials, f)
print(opt_params)
    
# save trials history
trials_df = make_trials_df(trials)
trials_df.to_csv(os.path.join(home_dir, os.path.dirname(meta_file), 'trials.csv'), index=False)
fig = plot_trials(trials_df)
fig.savefig(os.path.join(home_dir, os.path.dirname(meta_file), 'trials.png'), bbox_inches='tight')

# clean up
shutil.rmtree(meta['temp_train_image_dir'])
shutil.rmtree(meta['temp_valid_image_dir']) 
    
#if __name__ == '__main__':
#    main()

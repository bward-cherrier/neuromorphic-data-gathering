# coding: utf-8

import os, math, time, shutil, pickle, warnings
warnings.filterwarnings("ignore")
from functools import partial
from hyperopt import tpe, hp, fmin, Trials
from hyperopt.pyll import scope

from experiments.TacTip_datasets.train_dnn_opt import make_model_meta, build_objective_func, plot_trials, make_trials_df

def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'plane3dTap')
    train_meta_file = os.path.join(home_dir, 'collect_tap_rand_efi_02031105', 'meta.json')
    valid_meta_file = os.path.join(home_dir, 'collect_tap_rand_efi_02031304', 'meta.json')
    
    # hyperopt search parameters
    space = {
        'num_conv_layers': scope.int(hp.quniform(label='num_conv_layers', low=1, high=5, q=1)),
        'num_conv_filters': scope.int(2 ** (hp.quniform(label='num_conv_filters', low=1, high=9, q=1))),
        'num_dense_layers': scope.int(hp.quniform(label='num_dense_layers', low=1, high=5, q=1)),
        'num_dense_units': scope.int(2 ** (hp.quniform(label='num_dense_units', low=1, high=9, q=1))),
        'activation':  hp.choice(label='activation', options=('relu', 'elu')),
        'dropout': hp.uniform(label='dropout', low=0, high=0.5),
        'kernel_l1': hp.loguniform(label='kernel_l1', low=-4*math.log(10), high=-math.log(10)),
        'kernel_l2': hp.loguniform(label='kernel_l2', low=-4*math.log(10), high=-math.log(10)),
        'batch_size': scope.int(2 ** (hp.quniform(label='batch_size', low=4, high=7, q=1)))}
    max_evals = 100
    n_startup_jobs = 50
    
    # make meta data
    model_dirname = os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.path.dirname(train_meta_file), 'models', model_dirname, 'meta.json')
    meta = make_model_meta(meta_file, train_meta_file, valid_meta_file,
                           epochs = 200,
                           patience = 100,
                           loss_weights = [1/5**2, 1/15**2, 1/15**2],
                           target_names = ['pose_3', 'pose_4', 'pose_5'])
    os.makedirs(os.path.dirname(meta_file))
    
    # unpack images to temporary folder
    meta['temp_train_image_dir'] = os.environ['TEMPPATH'] + r'\train'+time.strftime('%m%d%H%M')
    meta['temp_valid_image_dir'] = os.environ['TEMPPATH'] + r'\valid'+time.strftime('%m%d%H%M')
    shutil.unpack_archive(meta['train_image_dir']+r'.zip', meta['temp_train_image_dir'])
    shutil.unpack_archive(meta['valid_image_dir']+r'.zip', meta['temp_valid_image_dir'])
            
    # perform optimization
    trials = Trials()
    obj_func = build_objective_func(meta)
    opt_params = fmin(obj_func, space, max_evals=max_evals, trials=trials, 
                      algo=partial(tpe.suggest, n_startup_jobs=n_startup_jobs))
    with open(os.path.join(os.path.dirname(meta_file), "trials.pickle"), "wb") as f:
        pickle.dump(trials, f)
    print(opt_params)
        
    # save trials history
    trials_df = make_trials_df(trials)
    trials_df.to_csv(os.path.join(os.path.dirname(meta_file), 'trials.csv'), index=False)
    fig = plot_trials(trials_df)
    fig.savefig(os.path.dirname(meta_file) + r'\trials.png', bbox_inches='tight')
    
    # clean up
    shutil.rmtree(meta['temp_train_image_dir'])
    shutil.rmtree(meta['temp_valid_image_dir']) 
    
if __name__ == '__main__':
    main()

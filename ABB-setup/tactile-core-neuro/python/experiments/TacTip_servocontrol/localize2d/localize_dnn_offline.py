# -*- coding: utf-8 -*-
  
import os, time, json, shutil, warnings, cv2, math
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from copy import copy
from win32api import GetSystemMetrics
from core.model.dnn.CNN_model import CNNmodel

def make_test_meta(meta_file, test_meta_file, model_meta_file,
                   n_steps = 20,
                   r = [0, 0, -1, 0, 0, 0],
                   kp = [0.5, 0, 0.5, 0.5, 0.5, 0.5],
                   ki = [0.5, 0, 0.5, 0.5, 0.5, 0.5],
                   init_pose = [-4, 0, -2, -15, -15, 30],
                   ):
    data_dir = os.path.dirname(meta_file)
    test_dir = os.path.dirname(test_meta_file)
    model_dir = os.path.dirname(model_meta_file)
    
    # load metadata
    with open(model_meta_file, 'r') as f: 
        meta = json.load(f)    
        
    # update metadata
    test_image_dir = os.path.join(test_dir, 'images_bw')
    test_df_file = os.path.join(test_dir, 'targets_image.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
    data_image_dir = os.path.join(data_dir, 'images_bw')
    
    # ensure appropriate type
    size = tuple(meta['size']) 
    
    # return metadata
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['test_meta_file'], meta['model_meta_file']
    del meta['data_dir'], meta['model_dir'], meta['test_dir']
    return meta

def control_dnn_offline(model, test_image_dir, test_df_file, target_names, data_image_dir,
                n_steps = 10, 
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-math.inf,]*6,[math.inf,]*6],
                init_pose = [0,]*6,
                **kwargs):    
    os.makedirs(data_image_dir)
    test_df = pd.read_csv(test_df_file)

    y = np.empty((n_steps+1, 6))
    e = np.empty((n_steps+1, 6))
    ei = np.empty((n_steps+1, 6))
    u = np.empty((n_steps+1, 6))
    ei[0,] = [0,]*6
    u[0,] = copy(r)
    y[0,] = r - np.array(init_pose)
    
    pose = ['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']
    target_inds = [pose.index(t) for t in target_names]
    h = PlotControl(target_names, ['prediction', 'error', 'integrated error'])
    
    # step method
    pred_df = pd.DataFrame()
    for i in range(n_steps):
        data_image_file = os.path.join(data_image_dir, f'image_{i+1}.jpg') 
            
        # control signal in sensor frame
        e[i,] = r - y[i,]
        ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,],ei_bnd[0]), ei_bnd[1])
        u[i+1,] = kp*e[i,] + ki*ei[i+1,] 
    
        # Model prediction
        i_target = np.argmin( np.sum( (test_df[pose] - u[i+1,])**2, axis=1), axis=0)
        test_image_file = os.path.join(test_image_dir, test_df['sensor_image'][i_target])
        image = cv2.imread(test_image_file)
        cv2.imwrite(data_image_file, image)
    
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = image.astype('float32') / 255
        image = image[np.newaxis, ..., np.newaxis]
        y[i+1,target_inds] = model.predict(image)
    
        with np.printoptions(precision=2, suppress=True):
            print(f'{i+1}: y={y[i+1,]}, e={e[i,]}')
    
        pred_row = pd.Series({'sensor_image': os.path.basename(data_image_file)})
        for j, jtem in enumerate(pose):
            pred_row['pose_'+str(j+1)] = u[i+1,j]
            pred_row['pred_'+str(j+1)] = y[i+1,j]
        pred_df = pred_df.append(pred_row, ignore_index=True)
        
        h.update(i+1, [y[i+1,target_inds], e[i,target_inds], ei[i+1,target_inds]])
        
    h.finish()
    return pred_df

class PlotControl:
    def __init__(self, target_names, row_names):
        plt.ion()
        nrows, ncols = len(row_names), len(target_names)
        self._fig, self._axs = plt.subplots(nrows, ncols, sharex=True,
                                           figsize=(3.5*ncols,3.5*nrows), num=1)
        self._fig.subplots_adjust(wspace=0.3)
        for i, ax in enumerate(self._axs.ravel()): 
            ax.plot([], [], '-r')  
            ax.grid()
            if i<ncols: ax.set(title=target_names[i%ncols])
            if i%ncols==0: ax.set(ylabel=row_names[i//ncols])
            if i//ncols==nrows-1: ax.set(xlabel='time steps')

        geom = self._fig.canvas.manager.window.geometry()      
        x,y,dx,dy = geom.getRect()
        sx,sy = GetSystemMetrics(0), GetSystemMetrics(1)
        self._fig.canvas.manager.window.setGeometry(sx-dx,sy-dy,dx,dy)
        plt.draw()
        
    def update(self, x, y):
        nrows, ncols = self._axs.shape
        for i, ax in enumerate(self._axs.ravel()):
            xd = np.append(ax.lines[0].get_xdata(), x)
            yd = np.append(ax.lines[0].get_ydata(), y[i//ncols][i%ncols])
            ax.set(xlim=[0.5,x+0.5])
            ax.set(ylim=[np.min(yd)-0.5,np.max(yd)+0.5])
            ax.lines[0].set(xdata=xd, ydata=yd)
        plt.draw()
        plt.pause(0.001)

    def finish(self):
        plt.show(block=True)


def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets\edge5dTap')
    test_meta_file = os.path.join(home_dir, 'collectTapRand5d06200642\meta.json')
    model_meta_file = os.path.join(home_dir, r'collectTapRand5d06200640\models\trainDNNopt_11260921\meta_261.json')

    # Make test meta
    test_home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol\localize');
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(test_home_dir, test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, test_meta_file, model_meta_file)

    os.makedirs(os.path.dirname(meta_file), exist_ok=True)
    with open(meta_file, 'w') as f:
        json.dump(meta, f)

    # unpack to temporary folder
    temp_meta = {**meta, 'test_image_dir': os.environ['TEMPPATH'] + r'\test'+time.strftime('%m%d%H%M')}
    shutil.unpack_archive(meta['test_image_dir']+r'.zip', temp_meta['test_image_dir'])

    # startup/load model and make predictions on test data
    cnn = CNNmodel()
    cnn.load_model(**temp_meta)
    cnn.print_model_summary()

    # Collect data
    pred_df = control_dnn_offline(cnn, **temp_meta)
    pred_df.to_csv(os.path.dirname(meta_file) +r'\predictions.csv')

    # Tidy up
    shutil.rmtree(temp_meta['test_image_dir'])

if __name__ == '__main__':
    main()


#test_image_dir = temp_meta['test_image_dir']
#test_df_file = temp_meta['test_df_file']
#target_names = temp_meta['target_names']
#data_image_dir = temp_meta['data_image_dir']
#
#n_steps = meta['n_steps']
#r = [0,]*6
#kp = meta['kp']
#ki = meta['ki']
#ei_bnd = [[-math.inf,]*6,[math.inf,]*6]
#init_pose = meta['init_pose']

# from IPython import embed; embed()

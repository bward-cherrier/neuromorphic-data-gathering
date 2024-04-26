# -*- coding: utf-8 -*-
  
import os, json, warnings, cv2, math
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
from copy import copy
from core.model.dnn.CNN_model import CNNmodel
from localize_dnn_offline import PlotControl


def control_dnn_replay(model, data_image_dir, target_names, 
                n_steps = 10, 
                r = [0,]*6,
                kp = [0.5,]*6,
                ki = [0,]*6,
                ei_bnd = [[-math.inf,]*6,[math.inf,]*6],
                init_pose = [0,]*6,
                **kwargs):    
        
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
        test_image_file = os.path.join(data_image_dir, f'image_{i+1}.jpg') 
            
        # control signal in sensor frame
        e[i,] = r - y[i,]
        ei[i+1,] = np.minimum(np.maximum(e[i,]+ei[i,],ei_bnd[0]), ei_bnd[1])
        u[i+1,] = kp*e[i,] + ki*ei[i+1,] 
    
        # Model prediction
        image = cv2.imread(test_image_file)
    
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = image.astype('float32') / 255
        image = image[np.newaxis, ..., np.newaxis]
        y[i+1,target_inds] = model.predict(image)
    
        with np.printoptions(precision=2, suppress=True):
            print(f'{i+1}: y={y[i+1,]}, e={e[i,]}')
    
        pred_row = pd.Series({'sensor_image': os.path.basename(test_image_file)})
        for j, jtem in enumerate(pose):
            pred_row['pose_'+str(j+1)] = u[i+1,j]
            pred_row['pred_'+str(j+1)] = y[i+1,j]
        pred_df = pred_df.append(pred_row, ignore_index=True)
        
        h.update(i+1, [y[i+1,target_inds], e[i,target_inds], ei[i+1,target_inds]])

    h.finish()
    return pred_df    


def main():    
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol\localize')
    replay_meta_file = os.path.join(home_dir, 'localize_dnn_offline_01050633\meta.json')
     
    with open(replay_meta_file, 'r') as f: 
        meta = json.load(f)    

    # startup/load model and make predictions on test data
    cnn = CNNmodel()
    cnn.load_model(**meta)
    cnn.print_model_summary()
       
    # replay data
    control_dnn_replay(cnn, **meta)
    plt.show(block=True)

if __name__ == '__main__':
    main()


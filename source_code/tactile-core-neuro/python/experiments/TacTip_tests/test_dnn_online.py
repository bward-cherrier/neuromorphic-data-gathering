# -*- coding: utf-8  v  -*-
  
import os, json, warnings, time, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
from core.model.dnn.CNN_model import CNNmodel
from experiments.TacTip_datasets.extract_images import process_image

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvImageOutputFileSeq
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor


def make_sensor():
    camera=CvVideoCamera(source=0, exposure=-6)
    for _ in range(5): 
        camera.read() # Hack - camera transient ABB1
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=camera,
            display=CvVideoDisplay(name='preview'),
            writer=CvImageOutputFileSeq(start_frame=1),
        ))
    

def make_test_meta(meta_file, model_meta_file,
                   num_steps = 10,
                   ):
    test_dir =  os.path.dirname(meta_file)
    model_dir =  os.path.dirname(model_meta_file)
        
    # load metadata
    with open(model_meta_file, 'r') as f: 
        meta = json.load(f)

    # update metadata
    test_image_dir = os.path.join(test_dir, 'images')
    test_df_file = os.path.join(test_dir, 'predictions.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
    
    # ensure appropriate type
    size = tuple(meta['size']) 
    
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'], meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta

def test_dnn(model, test_image_dir, target_names,
               num_frames, crop, size, threshold, 
               num_steps = 10,
               **kwargs):         
    y = np.zeros((num_steps+1, 6))

    os.makedirs(test_image_dir)
    pose = ['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']
    target_inds = [pose.index(t) for t in target_names]
    
    with make_sensor() as sensor:       
        # step method
        pred_df = pd.DataFrame()
        for i in range(num_steps):
            test_file = os.path.join(test_image_dir, f'image_{i+1}.png')
                
            # capture data
            image = sensor.process(num_frames=1+num_frames, outfile=test_file)
            image = process_image(image[1,:,:,:], crop, size, threshold)
            image = image[np.newaxis, ...].astype('float32') / 255
            
            # make prediction
            y[i+1,target_inds] = model.predict(image)    
            with np.printoptions(precision=2, suppress=True):
                print(f'{i+1}: y={y[i+1,]}')
        
            pred_row = pd.Series({'sensor_video': os.path.basename(test_file)})
            for j, jtem in enumerate(pose):
                pred_row['pred_'+str(j+1)] = y[i+1,j]
            pred_df = pred_df.append(pred_row, ignore_index=True)
            
            # Wait 
            time.sleep(1)

    return pred_df     

      
def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'plane3d')
    model_meta_file = os.path.join(home_dir, 'collect_rand_3d_01271645', 'models', 'train_dnn_01271716', 'meta.json')

    # Make test meta
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.environ['DATAPATH'], 'TacTip_tests', test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, model_meta_file)
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
            json.dump(meta, f)

    # temporary image folder
    temp_meta = {**meta, 'test_image_dir': os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))}

    # startup/load model
    cnn = CNNmodel()
    cnn.load_model(**temp_meta)
    cnn.print_model_summary()
  
    # Collect data
    pred_df = test_dnn(cnn, **temp_meta)
    pred_df.to_csv(temp_meta['test_df_file'])
    
    # Tidy up
    shutil.make_archive(meta['test_image_dir'], 'zip', temp_meta['test_image_dir'])
    shutil.rmtree(temp_meta['test_image_dir'])

if __name__ == '__main__':
    main()

# -*- coding: utf-8  v  -*-
  
import os, json, warnings, time, shutil
warnings.filterwarnings("ignore")
import pandas as pd
import numpy as np
#from core.model.dnn.CNN_model import CNNmodel
#from experiments.TacTip_datasets.extract_images import process_image

#from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
#from vsp.processor import CameraStreamProcessorMT, AsyncProcessor

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvVideoOutputFile
from vsp.detector import CvBlobDetector
from vsp.tracker import NearestNeighbourTracker
from vsp.processor import CameraStreamProcessorMT, AsyncProcessor
from vsp.encoder import KeypointEncoder
from vsp.view import KeypointView

def make_sensor():
    return AsyncProcessor(CameraStreamProcessorMT(
            camera=CvVideoCamera(exposure=-6),
            pipeline=[
                CvBlobDetector(
                    min_threshold=33,
                    max_threshold=199,
                    filter_by_color=True,
                    blob_color=255,
                    filter_by_area=True,
                    min_area=23.4,
                    max_area=119,
                    filter_by_circularity=True,
                    min_circularity=0.42,
                    filter_by_inertia=True,
                    min_inertia_ratio=0.29,
                    filter_by_convexity=True,
                    min_convexity=0.57,           
                ),
                NearestNeighbourTracker(threshold=20),
                KeypointEncoder(),
            ],
            view=KeypointView(color=(0,255,0)),
            display=CvVideoDisplay(name='preview'),
            writer=CvVideoOutputFile(),
        ))

def make_test_meta(meta_file, model_meta_file,
                   num_steps = 100,
                   ):
    test_dir =  os.path.dirname(meta_file)
    model_dir =  os.path.dirname(model_meta_file)
        
    # load metadata
    with open(model_meta_file, 'r') as f: 
        meta = json.load(f)

    # update metadata
    test_video_dir = os.path.join(test_dir, 'videos')
    test_df_file = os.path.join(test_dir, 'predictions.csv')
    model_file = os.path.join(model_dir, os.path.basename(meta['model_file']))
    
    # ensure appropriate type
    size = tuple(meta['size']) 
    
    # save metadata in test dir
    meta.update(locals().copy())
    del meta['meta'], meta['f'],  meta['model_meta_file']
    del meta['model_dir'], meta['test_dir']
    return meta

def test_dnn(test_video_dir, target_names,
               num_frames, crop, size, threshold, 
               num_steps = 10,
               **kwargs):         
#    y = np.zeros((num_steps+1, 6))

    os.makedirs(test_video_dir)
#    pose = ['pose_1','pose_2','pose_3','pose_4','pose_5','pose_6']
#    target_inds = [pose.index(t) for t in target_names]
    
    with make_sensor() as sensor:       
        # step method
        pred_df = pd.DataFrame()
        for i in range(num_steps):
            test_file = os.path.join(test_video_dir, f'video_{i+1}.mp4')
                
            # capture data
            kpts = sensor.process(num_frames=10, outfile=test_file )
            print(kpts.shape)

#            image = sensor.process(num_frames=1+num_frames, outfile=test_file)
#            image = process_image(image[1,:,:,:], crop, size, threshold)
#            image = image[np.newaxis, ...].astype('float32') / 255
            
            # make prediction
#            y[i+1,target_inds] = model.predict(image)    
#            with np.printoptions(precision=2, suppress=True):
#                print(f'{i+1}: y={y[i+1,]}')
#        
#            pred_row = pd.Series({'sensor_video': os.path.basename(test_file)})
#            for j, jtem in enumerate(pose):
#                pred_row['pred_'+str(j+1)] = y[i+1,j]
#            pred_df = pred_df.append(pred_row, ignore_index=True)
#            
            # Wait 
            time.sleep(1)

    return pred_df     

      
def main():
    # Specify directories and files - to edit
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'plane3d')
    model_meta_file = os.path.join(home_dir, 'collect_rand_3d_01271645', 'models', 'train_dnn_01271716', 'meta.json')

    # Make test meta
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.environ['DATAPATH'], 'TacTip_servocontrol', 'tests', test_dirname, 'meta.json')
    meta = make_test_meta(meta_file, model_meta_file)
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
            json.dump(meta, f)

    # temporary image folder
    temp_meta = {**meta, 'test_video_dir': os.path.join(os.environ['TEMPPATH'], 'videos'+time.strftime('%m%d%H%M'))}

#    # startup/load model
#    cnn = CNNmodel()
#    cnn.load_model(**temp_meta)
#    cnn.print_model_summary()
    
    
    # Collect data
    pred_df = test_dnn(**temp_meta)
#    pred_df.to_csv(temp_meta['test_df_file'])
    
    # Tidy up
    shutil.make_archive(meta['test_video_dir'], 'zip', temp_meta['test_video_dir'])
    shutil.rmtree(temp_meta['test_video_dir'])

if __name__ == '__main__':
    main()

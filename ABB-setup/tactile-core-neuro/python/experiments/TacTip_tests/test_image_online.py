# -*- coding: utf-8  v  -*-
  
import os, json, warnings, time, shutil
warnings.filterwarnings("ignore")
import numpy as np
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


def test_sensor(test_image_dir, 
               num_frames, crop, size, threshold, 
               num_steps = 10,
               **kwargs):         

    os.makedirs(test_image_dir)

    with make_sensor() as sensor:       
        # step method
        for i in range(num_steps):
            test_file = os.path.join(test_image_dir, f'image_{i+1}.png')
                
            # capture data
            image = sensor.process(num_frames=1+num_frames, outfile=test_file)                      
            image = process_image(image[1,:,:,:], crop, size, threshold)
            image = image[np.newaxis, ...].astype('float32') / 255

            # Wait 
            time.sleep(1)
            
      
def main():
    # Specify directories and files - to edit
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.environ['DATAPATH'], 'TacTip_tests', test_dirname, 'meta.json')
    
    meta = {'test_image_dir': os.path.join(os.path.dirname(meta_file), 'images'),
            'num_frames': 2,
            'num_steps': 10,
            'size': (128, 128),
            'crop': (160, 70, 490, 400),
            'threshold': (11, -5)}
    
    os.makedirs(os.path.dirname(meta_file))
    with open(meta_file, 'w') as f: 
            json.dump(meta, f)

    # temporary image folder
    temp_meta = {**meta, 'test_image_dir': os.path.join(os.environ['TEMPPATH'], 'images'+time.strftime('%m%d%H%M'))}
  
    # Collect data
    test_sensor(**temp_meta)
    
    # Tidy up
    shutil.make_archive(meta['test_image_dir'], 'zip', temp_meta['test_image_dir'])
    shutil.rmtree(temp_meta['test_image_dir'])

if __name__ == '__main__':
    main()

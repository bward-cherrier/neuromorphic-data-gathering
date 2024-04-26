# -*- coding: utf-8  v  -*-
  
import os, json, warnings, time, shutil
warnings.filterwarnings("ignore")

from vsp.video_stream import CvVideoCamera, CvVideoDisplay, CvImageOutputFileSeq
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
            writer=CvImageOutputFileSeq(start_frame=1),
        ))


def test_sensor(test_image_dir, 
               num_frames, 
               num_steps = 10,
               **kwargs):         

    os.makedirs(test_image_dir)

    with make_sensor() as sensor:       
        # step method
        for i in range(num_steps):
            test_file = os.path.join(test_image_dir, f'image_{i+1}.png')
                
            # capture data
            pins = sensor.process(num_frames=1+num_frames, outfile=test_file)                      
            print(pins)    
            
            # Wait 
            time.sleep(1)
            
      
def main():
    # Specify directories and files - to edit
    test_dirname = os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M')
    meta_file = os.path.join(os.environ['DATAPATH'], 'TacTip_tests', test_dirname, 'meta.json')
    
    meta = {'test_image_dir': os.path.join(os.path.dirname(meta_file), 'images'),
            'num_frames': 2,
            'num_steps': 10}
    
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

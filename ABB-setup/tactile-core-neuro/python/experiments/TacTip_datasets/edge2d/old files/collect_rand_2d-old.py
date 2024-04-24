# -*- coding: utf-8 -*-
    
import os, time, json, shutil
from experiments.TacTip_datasets.collect_rand import make_meta, make_video_df, collect
    

def main():    
    # Specify directories and files
    home_dir = os.path.join(os.environ['DATAPATH'], 'TacTip_datasets', 'edge2d')
    meta_file = os.path.join(home_dir, 
         os.path.basename(__file__)[:-3]+'_'+time.strftime('%m%d%H%M'), 'meta.json')

    # make meta data and dataframe
    meta = make_meta(meta_file,
                     ip = '164.11.73.64',
                     work_frame = [158, 392, 90, 180, 0, 180],
                     robot_tcp = [0, 0, 89.1, 0, 0, -180],
                     poses_rng = [[-6, 0, -1, 0, 0, -45], [6, 0, 1, 0, 0, 45]], 
                     moves_rng = [[-5, -5, 0, -5, -5, -5], [5, 5, 0, 5, 5, 5]], 
                     num_poses = 500)    
    os.makedirs(os.path.dirname(meta_file))
    make_video_df(**meta)    
    with open(meta_file, 'w') as f: 
        json.dump(meta, f)   
    
    # temporary image folder
    temp_meta = {**meta, 'video_dir': os.path.join(os.environ['TEMPPATH'], 'videos'+time.strftime('%m%d%H%M'))}
    
    # collect data
    collect(**temp_meta)
    
    # tidy up
    shutil.make_archive(meta['video_dir'], 'zip', temp_meta['video_dir'])
    shutil.rmtree(temp_meta['video_dir'])
               
if __name__ == '__main__':
    main()

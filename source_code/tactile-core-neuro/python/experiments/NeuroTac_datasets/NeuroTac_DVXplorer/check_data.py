import numpy as np
import pickle

path = '/home/ben/Data/NeuroTac_DVXplorer/ABB_taps_positionNoise_Xingchen/ABB_taps_positionNoise_Xingchen_03151042/events/'
# path = '/home/ben/Data/NeuroTac_DVXplorer/camera_tests/camera_tests_03061530/events/'

name = 'taps_trial_0_pose_8_events_on'
# name = 'd2_s1.0_dir_right_r0_events_off'

with open(path+'/'+name, 'rb') as f:
    data = pickle.load(f)

print(data)
print(np.unique(data))
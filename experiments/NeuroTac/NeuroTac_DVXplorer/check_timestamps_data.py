import numpy as np
import pickle
import matplotlib.pyplot as plt

path = '/home/bw14452/Data/NeuroTac_DVXplorer/horizontal_shear/horizontal_shear_3_runs_timestamps'
# path = '/home/ben/Data/NeuroTac_DVXplorer/camera_tests/camera_tests_03061530/events/'

name = 'timestamps_tactip_out.pkl'
# name = 'd2_s1.0_dir_right_r0_events_off'

with open(path+'/'+name, 'rb') as f:
    data = pickle.load(f)

# print(data)
# print(data['tactip_out_d2_s2.0_dir_left_r0'])

plt.plot(data['tactip_out_d2_s2.0_dir_left_r0'])
plt.savefig('timestamps_example')
plt.show()
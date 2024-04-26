from pandas import read_hdf, HDFStore, DataFrame
import io, os
import time
import numpy as np
from datetime import datetime

def create_hdf(data_directory, filename='data'):
    ### Create new folder for data ###
    if os.path.isdir(data_directory) == False:
        os.makedirs(data_directory) # Create sub-directory for saving files
    save_directory = data_directory + '/' + filename + '.h5'
    hdf = HDFStore(save_directory)
    return hdf

def save_hdf_data(hdf,data,i=1,obj_idx=0,run_idx=0):
    hdf.open()
    cut_data = data[:i,:,:]
    cut_data = np.reshape(cut_data,(i*data.shape[1],7))
    hdf.append('obj'+str(obj_idx)+'run'+str(run_idx),DataFrame(cut_data, 
    columns=('px','py','ex','ey','ne','t','t_frame')), 
    format='table', data_columns=True)
    hdf.close()

<<<<<<< HEAD
def save_hdf_data_speeds(hdf,data,i=1,obj_idx=0,spd_idx = 0,run_idx=0):
    hdf.open()
    cut_data = data[:i,:,:]
    cut_data = np.reshape(cut_data,(i*data.shape[1],7))
    hdf.append('obj'+str(obj_idx)+'speed'+str(spd_idx) +'run'+str(run_idx),DataFrame(cut_data, 
    columns=('px','py','ex','ey','ne','t','t_frame')), 
    format='table', data_columns=True)
    hdf.close()

=======
>>>>>>> 169db60179ea8d4563cc2aee68dca4dfe49f13ef
###############
## LOAD DATA ##
###############

def load_hdf_data(data_directory, obj_idx, run_idx, n_pins):
    data = read_hdf(data_directory + '/data.h5','obj'+str(obj_idx)+'run'+str(run_idx),columns=['px','py','ex','ey','ne','t','t_frame'])
    data = np.array(data)
    length = data.shape[0]
    data = data.reshape(int(length/n_pins),n_pins,7)
    return data

def load_hdf_data_old(data_directory, run_idx, n_pins):
    data = read_hdf(data_directory + '/data.h5','d'+str(run_idx),columns=['px','py','ex','ey','ne','t','t_frame'])
    data = np.array(data)
    length = data.shape[0]
    data = data.reshape(int(length/n_pins),n_pins,7)
    return data
<<<<<<< HEAD
    
def load_hdf_data_speeds (data_directory, obj_idx, run_idx, spd_idx, n_pins):
    data = read_hdf(data_directory + '/data.h5','obj'+str(obj_idx)+'speed'+str(spd_idx)+'run'+str(run_idx),columns=['px','py','ex','ey','ne','t','t_frame'])
    data = np.array(data)
    length = data.shape[0]
    data = data.reshape(int(length/n_pins),n_pins,7)
    return data

=======
>>>>>>> 169db60179ea8d4563cc2aee68dca4dfe49f13ef

################
## LABEL DATA ##
################

# def label_hdf():
    # def auto_label(date='2602',time='1138',sensors=[1],n_runs=50,raw = False):
    # n_sensors = len(sensors)
    # data_dict = {}
    # slip_inds = np.ndarray((n_runs,n_sensors,2)).astype(int)
    # for run in range(n_runs):
    #     for v,i in enumerate(sensors):
    #         if raw:
    #             data = read_hdf(date+time+'/f'+str(i)+'.h5','d'+str(run),columns=['t'])
    #             data = np.array(data)
    #             data -= data[0]
    #         else:
    #             data = read_hdf(date+time+'/f'+str(i)+'.h5','d'+str(run),columns=['x','y','t'])
    #             data = np.array(data)
    #             data = data[::30,2]
    #             data -= data[0] 
            
    #         frames = data.shape[0]

    #         a_data = read_hdf(date+time+'/aruco.h5','d'+str(run),columns=['h','t'])
    #         a_data = np.array(a_data)
    #         a_frames = a_data.shape[0]

    #         ## Find max of aruco
    #         search_range = [200,a_frames-30]

    #         top = np.amax(a_data[search_range[0]:search_range[1],0])
    #         top_i = np.argmax(a_data[search_range[0]:search_range[1],0])+search_range[0]
    #         falling = False
    #         for s in range(top_i+1,search_range[1]):
    #             comparison = a_data[s-1,0]
    #             if a_data[s,0] < comparison and a_data[s,0] != 0:
    #                 if a_data[s,0] == comparison-1:
    #                     lookahead = 1
    #                     while True:
    #                         if a_data[s+lookahead,0] == 0:
    #                             # No aruco reading
    #                             lookahead += 1
    #                         elif a_data[s+lookahead,0] < a_data[s,0]:
    #                             # Fall start
    #                             falling = True
    #                             break
    #                             continue
    #                         else:
    #                             break
    #                 else:
    #                     falling = True
    #             else:
    #                 continue

    #             if falling:
    #                 break

    #         slip_time = a_data[s,1]
    #         print(run,s)
    #         pos_ind = np.nonzero(data>slip_time)[0][0]-1 # -1 because need to do it for velocities which have indicies shifted by 1
    #         slip_inds[run,v] = [pos_ind,pos_ind+10]

    # return slip_inds
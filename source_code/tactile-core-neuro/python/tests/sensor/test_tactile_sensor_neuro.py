
import time 
import numpy as np
from data_operations import create_hdf, save_hdf, save_init_taxels_hdf
from tactile_sensor_neuro import TacTip_neuro

# Create hd5 files for data storage
hdf_init = create_hdf('initPositions')
hdf_data = create_hdf('data')

# Start sensor
sensor = TacTip_neuro()

# Save initial taxel positions
init_taxel_positions = sensor.get_init_taxel_positions()
save_init_taxels_hdf(hdf_init,init_taxel_positions)

# Set variables for data collection
data_save_index = 0
nDataPoints = 7
data = np.zeros((10000,sensor.n_pins,nDataPoints-1), dtype=np.int32)

# Get data 
sensor.start_recording()
while sensor.n_frame < 1000:
	taxel_events = sensor.get_taxel_events()
	if len(taxel_events)!=0:
		for te in taxel_events:
			data[sensor.n_frame,te[0],:] = te[1:]
	time.sleep(.01)
sensor.stop_recording()

# Save data
save_hdf(hdf_data,data,sensor.n_frame,data_save_index)

# Close connection to sensor
sensor.close()


import numpy as np
import time, pickle
import h5py 

from dv import NetworkEventInput 
import dv_processing as dv
import datetime
import cv2 as cv
import os

################# The NeuroTac class runs using the dv-processing software from inivation ################################

class NeuroTac:
    def __enter__(self):
        return self

    def __exit__(self, type, value, tb)-> None:
        self.close()

    # Initialize sensor variables
    def __init__(self, save_events_video=False, save_acc_video=False, display = False)-> None:
        self.camera_type = dv.io.discoverDevices()[0]    # Select first detected camera
        self.camera = dv.io.CameraCapture(self.camera_type)
        self.noise_filter = dv.noise.BackgroundActivityNoiseFilter(self.camera.getEventResolution(), backgroundActivityDuration=datetime.timedelta(milliseconds=10))
        self.n_data_points = (self.camera.getEventResolution()[::-1]) # NOTE: Gross inversing of resolution tuple as DV and Numpy differ on how they designate x and y coordinates
        self.event_store = dv.EventStore()
        self.events_on = np.empty(self.n_data_points, dtype=object)
        self.events_off = np.empty(self.n_data_points, dtype=object)
        self.frames = []
        self.frames_filename = None
        self.thread_run = True
        self.starttime = 0  # DV sensor time at which the data gathering starts
        self.endtime = None
        self.save_events_video = save_events_video
        self.save_acc_video = save_acc_video
        self.events_on_filename = None
        self.events_off_filename = None
        self.events_stream_filename = None
        self.events_video_filename = None
        self.acc_video_filename = None
        self.display = display

    # Reset ON and OFF events to arrays containing empty lists and empty the frames list
    def reset_variables(self) -> None:
        self.frames = []
        self.event_store = dv.EventStore()

        for iy, ix in np.ndindex(self.n_data_points):
            self.events_on[iy,ix] = []
            self.events_off[iy,ix] = []

    def set_filenames(self, events_on_file = None, events_off_file = None, events_stream_file = None, events_video_file = None, acc_video_file = None, frames = None) -> None:
        self.events_on_filename = events_on_file
        self.events_off_filename = events_off_file
        self.events_stream_filename = events_stream_file
        self.events_video_filename = events_video_file
        self.acc_video_filename = acc_video_file
        self.frames_filename = frames
        
    # Start logging data
    def start_logging(self):
        self.thread_run = True
        print('Started recording')

    # Stop logging data
    def stop_logging(self):
        self.thread_run = False
        print('Stopped recording')  
            
    # Log ON and OFF pixel events
    def get_events(self) -> None: 

        # camera = dv.io.CameraCapture(self.camera_type)
        # Display events in a cv2 window if display_events flag has been set      
        if self.save_events_video:
            out = cv.VideoWriter(self.events_video_filename ,cv.VideoWriter_fourcc('m','p','4','v'), 30, self.camera.getEventResolution())
            visualizer = dv.visualization.EventVisualizer(self.camera.getEventResolution())
            # Create window and callback function for displaying events
            # cv.namedWindow("Preview events", cv.WINDOW_AUTOSIZE)
            def preview_events(event_slice):
                frame = visualizer.generateImage(event_slice)
                out.write(frame)
                if self.display:
                    cv.imshow('Preview events', frame)
                    cv.waitKey(2)
            # Create an event slicer, this will only be used events only camera
            slicer_events = dv.EventStreamSlicer()
            slicer_events.doEveryTimeInterval(datetime.timedelta(milliseconds=33), preview_events)
            
            
        # Display accumulator in a cv2 window if display_acc flag is set 
        if self.save_acc_video:
            out_acc = cv.VideoWriter(self.acc_video_filename , cv.VideoWriter_fourcc('m','p','4','v'), 30, self.camera.getEventResolution(),False)
            accumulator = dv.Accumulator(self.camera.getEventResolution())
            accumulator.setEventContribution(0.2)
            accumulator.setNeutralPotential(0.0)
            accumulator.setMinPotential(0.0)
            accumulator.setMaxPotential(1.0)
            accumulator.setDecayFunction(dv.Accumulator.Decay.LINEAR)
            accumulator.setDecayParam(1e-6)
            accumulator.setSynchronousDecay(False)
            accumulator.setRectifyPolarity(False)
            
             # Create the preview window and accumulator callback
            # cv.namedWindow("Preview accumulator", cv.WINDOW_AUTOSIZE)
            def accumulate_events(event_slice):
                accumulator.accept(event_slice)
                frame = accumulator.generateFrame()
                out_acc.write(frame.image)
                if self.display:
                    cv.imshow('Preview accumulator', frame.image)
                    cv.waitKey(2)

            # Create an event slicer, this will only be used events only camera
            slicer_acc = dv.EventStreamSlicer()
            slicer_acc.doEveryTimeInterval(datetime.timedelta(milliseconds=33), accumulate_events)

        self.starttime = dv.now()
        # recorded_first = False
        
        # Loop to grab events
        while self.camera.isRunning():     # Context manager will handle the closure of this infinite loop
            # Get events
            event_batch = self.camera.getNextEventBatch()
            
            # # If no events arrived yet, continue reading
            if event_batch is not None:
                event_batch = event_batch.sliceTime(self.starttime)
                self.noise_filter.accept(event_batch)
                filtered = self.noise_filter.generateEvents()
                self.event_store.add(filtered)             
                # if any(n < 0 for n in filtered.timestamps()-self.starttime) and not recorded_first:
                #     print(str((filtered.timestamps()-self.starttime)[0]))
                #     recorded_first = True
                        
                if self.save_events_video:
                    slicer_events.accept(filtered)
                if self.save_acc_video:
                    slicer_acc.accept(filtered)

            if self.thread_run == False:
                  self.endtime = dv.now()
                  return
    
    # Capture frames from camera. NOTE: Only to be used with compatible dvs cameras
    def get_frames(self) -> None:
        
        capture = dv.io.CameraCapture()

        while capture.isRunning():
            frame = capture.getNextFrame()

            if frame is not None:
                self.frames.append(frame.image)

            if self.thread_run == False:
                return

    # Convert event store object into numpy array containing lists of timestamps
    def value_cleanup(self) -> None:
        for event in self.event_store:
            ts = (event.timestamp() - self.starttime) // 1000 

            if event.polarity():    
                self.events_on[event.y(), event.x()].append(ts)
            else:
                self.events_off[event.y(), event.x()].append(ts)

    # Save ON events
    def save_events_on(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.events_on_filename
        if format == 'pkl':
            with open(datafile, 'wb') as pickle_events_on:
                pickle.dump(self.events_on, pickle_events_on)
        elif format == 'npy':
            np.save(datafile, self.events_on, allow_pickle=True)

    # Save OFF events
    def save_events_off(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.events_off_filename
        if format == 'pkl':
            with open(datafile, 'wb') as pickle_events_off:
                pickle.dump(self.events_off, pickle_events_off)
        elif format == 'npy':
            np.save(datafile, self.events_off, allow_pickle=True)

    # Save a stream of all events in h5 format (ps,ts,xs,ys)
    def save_events_stream(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.events_stream_filename
        with h5py.File(datafile, "w") as f:
            # Set event stream data
            dset = f.create_group("events", (4))
            dset = f.create_dataset("events/ps", data=self.event_store.polarities(), dtype='bool')
            dset = f.create_dataset("events/ts", data=self.event_store.timestamps()/1000000) # Save timestamps in seconds
            dset = f.create_dataset("events/xs", data=[coords[0] for coords in self.event_store.coordinates()], dtype = "int16")
            dset = f.create_dataset("events/ys", data=[coords[1] for coords in self.event_store.coordinates()], dtype = "int16")

            # Set attributes for this dataset
            f.attrs['t0'] = self.starttime/1000000
            f.attrs['tk'] = self.endtime/1000000
            f.attrs['duration'] = f.attrs['tk'] - f.attrs['t0'] 
            f.attrs['num_events'] = len(self.event_store)
            f.attrs['num_imgs'] = len(self.frames)

            f.attrs['num_neg'] = np.count_nonzero(self.event_store.polarities()==0)
            f.attrs['num_pos'] = np.count_nonzero(self.event_store.polarities())
            f.attrs['sensor_resolution'] = np.array(self.camera.getEventResolution(),dtype='int64')
            


    # Save captured frames
    def save_frames(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.frames_filename
        if format == 'pkl':
            with open(datafile, 'wb') as pickle_frames:
                pickle.dump(self.frames, pickle_frames)
        elif format == 'npy':
            np.save(datafile, np.array(self.frames), allow_pickle=True)


    # Return the starting time of data gathering in milliseconds
    def get_starttime(self) -> None:
        return self.starttime

    def close(self) -> None:
        pass


############################# The NeuroTac_DV class runs using the DV graphical software from inivation ###########################################################################################

class NeuroTac_DV:
    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()

    # Initialize sensor variables
    def __init__(self, port = 7777, camera_type = 'DVXplorer'):
        if camera_type == 'DVXplorer':
            self.n_data_points = (640,480) # Camera resolution
        elif camera_type == 'DAVIS240':
            self.n_data_points = (240,180)
        elif camera_type == 'eDVS':
            self.n_data_points = (128, 128)
        else:
            print('Camera type not recognized')
        self.events_on = []
        self.events_off = []
        self.events_on_filename = None
        self.events_off_filename = None
        self.thread_run = True
        self.port = port # TCP/IP port to connect DV module to the sensor
        self.starttime = 0 # System time at which the data gathering starts

    # Reset ON and OFF events to empty arrays
    # MAKE MORE EFFICIENT? 
    def reset_variables(self):
        self.events_on = np.empty((self.n_data_points), dtype = object)
        self.events_off = np.empty((self.n_data_points), dtype = object)
        # for ii in range (self.n_data_points[0]):
        #     for jj in range (self.n_data_points[1]):
        #         self.events_on[ii,jj] = []
        #         self.events_off[ii,jj] = []
        for ii in range(self.n_data_points[0]):
            self.events_on[ii] = [[]for _ in range(self.n_data_points[1])]
            self.events_off[ii] = [[]for _ in range(self.n_data_points[1])]

    def set_filenames(self, events_on_file = None, events_off_file = None) -> None:
        self.events_on_filename = events_on_file
        self.events_off_filename = events_off_file
        
    # Start logging data
    def start_logging(self):
        self.thread_run = True
        print('Started recording')

    # Stop logging data
    def stop_logging(self):
        self.thread_run = False
        print('Stopped recording')

    # Log ON and OFF pixel events
    def get_events(self): 
        self.starttime = int(time.time()*1000) # Record the starting time for data gathering in milliseconds
        with NetworkEventInput (address = 'localhost', port = self.port) as i:
            for event in i:
                if event.polarity:
                    self.events_on[event.x, event.y].append(event.timestamp) # Record ON events
                else:
                    self.events_off[event.x, event.y].append(event.timestamp) # Record OFF events
                if self.thread_run == False:
                    break
    
    # Convert camera timestamps from microseconds to milliseconds
    def convert_events_to_milliseconds(self):
        self.events_on = [[[spike_time//1000 for spike_time in pixel] for pixel in row] for row in self.events_on]
        self.events_off = [[[spike_time//1000 for spike_time in pixel] for pixel in row] for row in self.events_off]
    
    # Convert camera timestamps from microseconds to milliseconds and subtract starting time
    def value_cleanup(self):
        self.events_on = [[[spike_time//1000 - self.starttime for spike_time in pixel] for pixel in row] for row in self.events_on]
        self.events_off = [[[spike_time//1000 - self.starttime  for spike_time in pixel] for pixel in row] for row in self.events_off]

# Save ON events
    def save_events_on(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.events_on_filename
        if format == 'pkl':
            with open(datafile, 'wb') as pickle_events_on:
                pickle.dump(self.events_on, pickle_events_on)
        elif format == 'npy':
            np.save(datafile, self.events_on, allow_pickle=True)

    # Save OFF events
    def save_events_off(self, datafile:str = None, format = 'pkl') -> None:
        if datafile == None: 
            datafile = self.events_off_filename
        if format == 'pkl':
            with open(datafile, 'wb') as pickle_events_off:
                pickle.dump(self.events_off, pickle_events_off)
        elif format == 'npy':
            np.save(datafile, self.events_off, allow_pickle=True)
    
    # Return the starting time of data gathering in milliseconds
    def get_starttime(self):
        return self.starttime

    def close(self):
        pass

import dv_processing as dv
import numpy as np
import datetime
import cv2 as cv
import os


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
        self.thread_run = True
        self.starttime = 0  # DV sensor time at which the data gathering starts
        self.save_events_video = save_events_video
        self.save_acc_video = save_acc_video
        self.events_on_filename = None
        self.events_off_filename = None
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

    def set_filenames(self, events_on_file = None, events_off_file = None, events_video_file = None, acc_video_file = None) -> None:
        self.events_on_filename = events_on_file
        self.events_off_filename = events_off_file
        self.events_video_filename = events_video_file
        self.acc_video_filename = acc_video_file
        
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
    def save_events_on(self, datafile:str = None) -> None:
        if datafile == None: 
            datafile = self.events_on_filename
        np.save(datafile, self.events_on, allow_pickle=True)

    # Save OFF events
    def save_events_off(self, datafile:str = None) -> None:
        if datafile == None: 
            datafile = self.events_on_filename
        np.save(datafile, self.events_off, allow_pickle=True)

    # Save captured frames
    def save_frames(self, datafile:str) -> None:
        np.save(datafile, np.array(self.frames), allow_pickle=True)

    # Return the starting time of data gathering in milliseconds
    def get_starttime(self) -> None:
        return self.starttime

    def close(self) -> None:
        pass
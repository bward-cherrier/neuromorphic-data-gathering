# -*- coding: utf-8 -*-
"""
Created on Tue Nov  6 09:55:48 2018

@author: John
"""


import queue
import threading
import time
import cv2
import numpy as np
import scipy.spatial.distance as ssd

from datetime import datetime
<<<<<<< HEAD


=======


>>>>>>> 30df509f75cf062322ba26e0f9b220d7b2f9ef82
class TactileSensor:    
    def __init__(self, *args, **kwargs):
        
        # Initialise camera
        max_attempts = 5
        attempts = 1
        video_source = kwargs.get('video_source', 0)
        self._cap = cv2.VideoCapture(video_source)
        while not self._cap.isOpened() and attempts < max_attempts:
            self._cap = cv2.VideoCapture(video_source)
            attempts += 1
        if not self._cap.isOpened():
            raise RuntimeError('failed to initialise video source')
        dump_init_frames_count = 3
        for i in range(dump_init_frames_count):
            ret, frame = self._cap.read()
        
        # Set camera properties
        self._frame_width = kwargs.get('frame_width', 640)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
        self._frame_height = kwargs.get('frame_height', 480)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)        
        brightness = kwargs.get('brightness', 150)
        self._cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
        contrast = kwargs.get('contrast', 10)
        self._cap.set(cv2.CAP_PROP_CONTRAST, contrast)
        saturation = kwargs.get('saturation', 0)
        self._cap.set(cv2.CAP_PROP_SATURATION, saturation)
        exposure = kwargs.get('exposure', -6)
        self._cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

        # Set other camera-related parameters        
        self._camera_disp = kwargs.get('camera_disp', True)        
        self._camera_channel = kwargs.get('camera_channel', 3)

        # Set up frame display thread
        if self._camera_disp:
            self._display_bufsize = 10
            self._display_frames = queue.Queue(self._display_bufsize)
            self._display_interval = 0.01
            self._display = True
            self._display_thread = threading.Thread(target=self._display_frame, args=())
            self._display_thread.start()

        # Set up blob detector for tracking pins
        self._params = cv2.SimpleBlobDetector_Params()
        self._params.blobColor = 255
<<<<<<< HEAD
        self._params.thresholdStep = 5
        self._params.minRepeatability = 3
        self._params.minThreshold = kwargs.get('min_threshold', 50)
        self._params.maxThreshold = kwargs.get('max_threshold', 150)
=======
        self._params.minThreshold = kwargs.get('min_threshold', 58)
        self._params.maxThreshold = kwargs.get('max_threshold', 192)
>>>>>>> 30df509f75cf062322ba26e0f9b220d7b2f9ef82
        self._params.filterByArea = True
        self._params.minArea = kwargs.get('min_area', 21)
        self._params.maxArea = kwargs.get('max_area', 130)
        self._params.filterByCircularity = True        
        self._params.minCircularity = kwargs.get('min_circularity', 0.42)
        self._params.filterByConvexity = True
        self._params.minConvexity = kwargs.get('min_convexity', 0.48)
        self._params.filterByInertia = True        
        self._params.minInertiaRatio = kwargs.get('min_inertia_ratio', 0.27)
        self._detector = cv2.SimpleBlobDetector_create(self._params)
<<<<<<< HEAD
        print(self._params.thresholdStep)
=======
        print self._params.thresholdStep
>>>>>>> 169db60179ea8d4563cc2aee68dca4dfe49f13ef

        # Initialise pin positions  
        self._pin_tracking = kwargs.get('pin_tracking', True)
        if self._pin_tracking:
            self._max_tracking_move = kwargs.get('max_tracking_move', 20)
            max_pin_dist_from_centroid = kwargs.get('max_pin_dist_from_centroid', 300)
            min_pin_separation = kwargs.get('min_pin_separation', 0)
            self.init_pins(max_pin_dist_from_centroid, min_pin_separation)
    
    def close(self):
        if self._camera_disp:
            self._display = False
            self._display_thread.join()
        self._cap.release()
        
    def __del__(self):
        self.close()
        
    def _display_frame(self):
        while self._display:
            if not self._display_frames.empty():
                frame = self._display_frames.get()
                cv2.imshow('sensor', frame)
                cv2.waitKey(1)                
            time.sleep(self._display_interval)
        cv2.destroyAllWindows()

    def _select_pins(self, pins, max_dist_from_centroid, min_separation):
        # Calcualte centroid of pin positions 
        centroid = np.mean(pins, axis=0)
        centroid = centroid[np.newaxis,:]
        # Calculate distances of pins from centroid
        pin_rads = np.squeeze(ssd.cdist(pins, centroid, 'euclidean'))
        # Filter out pins that are further than specified distance from centroid
        pins = pins[pin_rads <= max_dist_from_centroid,:]        
        # Calculate distances between each pair of pins
        pin_dists = ssd.cdist(pins, pins, 'euclidean')
        processed = np.zeros(pins.shape[0], dtype=bool)
        selected = np.zeros(pins.shape[0], dtype=bool)
        # Add pin closest to centroid to selected pins
        idx = np.argmin(ssd.cdist(pins, centroid, 'euclidean'))
        selected[idx] = True
        while not np.all(processed):
            # Mark selected pins and pins closer than minimum separation as
            # processed
            processed = processed | selected | (pin_dists[idx,:] < min_separation)     
            pin_dists[idx, processed] = np.inf
            # Add closest unprocessed pin to selected pins
            idx = np.argmin(pin_dists[idx,:])
            selected[idx] = True
        pins = pins[selected,:]       
        # Sort pins by ascending x coordinate
        pins = pins[pins[:,0].argsort(),:]
        
        return pins

    def _map_pins(self, pins, prev_pins):
<<<<<<< HEAD
        # Map pin positions to closest matching previous position
        pin_dists = ssd.cdist(pins, prev_pins, 'euclidean')
        min_pin_idxs = np.argmin(pin_dists, axis=0)
        pins = pins[min_pin_idxs,:]
        # Overwrite with previous pin position if pin move exceeds threshold
        min_pin_dists = np.min(pin_dists, axis=0)        
        rep_pin_idxs = (min_pin_dists > self._max_tracking_move)
        pins[rep_pin_idxs,:] = prev_pins[rep_pin_idxs,:]
        
        return pins
=======
        # Map pins to closest previous pins as long as they are closer
        # than the max tracking move; otherwise retain the previous pins
        pin_dists = ssd.cdist(pins, prev_pins, 'euclidean')
        min_dists = np.min(pin_dists, axis=1)
        min_dist_idxs = np.argmin(pin_dists, axis=1)
        replace_pins = pins[min_dists < self._max_tracking_move]
        replace_idxs = min_dist_idxs[min_dists < self._max_tracking_move]
        new_pins = prev_pins
        new_pins[replace_idxs] = replace_pins        
        return new_pins
>>>>>>> 30df509f75cf062322ba26e0f9b220d7b2f9ef82

    def detect_pins(self, frame):
        # Capture frame-by-frame
        if frame is None:
            ret, frame = self._cap.read()
        # Our operations on the frame come here
        if self._camera_channel == 3:
            # Convert sensor frame to grayscale 
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:        
            # Convert sensor frame to b,g, or r 
            bgr = cv2.split(frame)
            frame = bgr[self._camera_channel]
        # Get keypoints and pin positions
        keypoints = self._detector.detect(frame)
        pins = np.array([k.pt for k in keypoints])

        return pins, keypoints
    
    def init_pins(self, max_pin_dist_from_centroid=300, min_pin_separation=0, filename=None):
        if filename is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(filename, fourcc, 20.0,
                                  (self._frame_width,self._frame_height))
        ret, frame = self._cap.read()
        if ret:
            if filename is not None: out.write(frame)
            pins, keypoints = self.detect_pins(frame)
            if self._camera_disp:
                im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                    (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                self._display_frames.put(im_with_keypoints)
            if pins.shape[0] == 0:
                raise RuntimeError('failed to identify any pins in frame')
            self._pins = self._select_pins(pins, max_pin_dist_from_centroid,
                                           min_pin_separation) 
        else:
            raise RuntimeError('failed to capture frame')
        if filename is not None: out.release()

        return self._pins

    def set_pins(self, pins):
        self._pins = pins
        
    def track_pins(self, filename=None, cancel=None, imshow=None):       
        if not self._pin_tracking:
            raise RuntimeError('cannot track pins with pin tracking disabled')
        
        if filename is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(filename, fourcc, 20.0,
                                  (self._frame_width,self._frame_height))
        while True:
            if cancel is not None and cancel(): break
            ret, frame = self._cap.read()
            if ret:
                if filename is not None: out.write(frame)
                pins, keypoints = self.detect_pins(frame)
                if pins.shape[0] == 0:
                    raise RuntimeError('failed to identify any pins in frame')
                self._pins = self._map_pins(pins, self._pins)
                if self._camera_disp:
                    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                        (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    self._display_frames.put(im_with_keypoints)
            else:
                raise RuntimeError('failed to capture frame')
        if filename is not None: out.release()       

    def record_pins(self, num_samples=1000000, filename=None, cancel=None):
        if filename is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(filename, fourcc, 20.0,
                                  (self._frame_width,self._frame_height))            
        pins_list = []
        for _ in range(num_samples):
            if cancel is not None and cancel(): break           
            ret, frame = self._cap.read()
            if ret:
                if filename is not None: out.write(frame)
                pins, keypoints = self.detect_pins(frame)
                if self._camera_disp:
                    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                        (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    self._display_frames.put(im_with_keypoints)
                if pins.shape[0] == 0:
                    continue
                    # raise RuntimeError('failed to identify any pins in frame')
                if self._pin_tracking:
                    self._pins = self._map_pins(pins, self._pins)
                else:
                    self._pins = pins
<<<<<<< HEAD
                    pins_list.append(self._pins)
=======
                   pins_list.append(self._pins)
>>>>>>> 169db60179ea8d4563cc2aee68dca4dfe49f13ef
            else:
                continue
                # raise RuntimeError('failed to capture frame')  
        pins_list = np.array(pins_list)
        if filename is not None: out.release()

        return pins_list

    def record_pins_with_time_stamp(self, num_samples=1000000, filename=None, cancel=None):
        if filename is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(filename, fourcc, 20.0,
                                  (self._frame_width,self._frame_height))            
        pins_list = []
        time_stamp_list = []
        for _ in range(num_samples):
            if cancel is not None and cancel(): break            
            ret, frame = self._cap.read()
            if ret:
                time_stamp_list.append(str(datetime.now()))
                if filename is not None: out.write(frame)
                pins, keypoints = self.detect_pins(frame)
                if self._camera_disp:
                    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                        (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    self._display_frames.put(im_with_keypoints)
                if pins.shape[0] == 0:
                    raise RuntimeError('failed to identify any pins in frame')
                self._pins = self._map_pins(pins, self._pins)
                pins_list.append(self._pins)
            else:
                raise RuntimeError('failed to capture frame')  
        pins_list = np.array(pins_list)
        if filename is not None: out.release()

        return (pins_list, time_stamp_list)        

    def record_frames(self, num_samples=1000000, filename=None, cancel=None):
        if filename is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(filename, fourcc, 20.0,
                                  (self._frame_width,self._frame_height))            
        frames = []
        for _ in range(num_samples):
            if cancel is not None and cancel(): break            
            ret, frame = self._cap.read()
            if ret:      
                if filename is not None: out.write(frame)
                if self._camera_disp:
                    self._display_frames.put(frame)
                frames.append(frame)
            else:
                raise RuntimeError('failed to capture frame')  
        frames = np.array(frames)
        if filename is not None: out.release()

        return frames

    def replay_frames(self, filename):
        frames = []
        cap = cv2.VideoCapture(filename)
        if cap.isOpened():
            not_eof = True
            while not_eof:
                not_eof, frame = cap.read()
                if not_eof:
                    frames.append(frame)
                    if self._camera_disp:
                        self._display_frames.put(frame)
        else:
            raise RuntimeError('failed to open video file')
        cap.release()  
        return np.array(frames)

    def replay_pins(self, filename):
        pins_list = []
        cap = cv2.VideoCapture(filename)
        if cap.isOpened():
            not_eof = True
            while not_eof:
                not_eof, frame = cap.read()
                if not_eof:
                    pins, keypoints = self.detect_pins(frame)
                    if self._camera_disp:
                        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                            (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                        self._display_frames.put(im_with_keypoints)
                    if pins.shape[0] == 0:
                        raise RuntimeError('failed to identify any pins in frame')
                    if self._pin_tracking:
                        self._pins = self._map_pins(pins, self._pins)
                    else:
                        self._pins = pins
                    pins_list.append(self._pins)
        else:
            raise RuntimeError('failed to open video file')
        cap.release()  
        return np.array(pins_list)


def main():
    sensor = TactileSensor(video_source=0,
                           brightness=150,
                           contrast=10,
                           saturation=0,
                           exposure=-6)  
    pins = sensor.record_pins(100)
    print(pins.shape)

                    
if __name__ == '__main__':
    main()

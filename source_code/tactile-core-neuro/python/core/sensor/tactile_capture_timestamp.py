import time
import numpy as np
import os, cv2, sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
from collections import deque
import pickle as pkl

from vsp.v4l2_camera import V4l2VideoCamera
from vsp.video_stream import CvVideoCamera, CvVideoDisplay
from vsp.processor import CameraStreamProcessorMT, CameraStreamProcessorMP, CameraStreamProcessor, Processor
from vsp.dataflow import DataflowQueueMT, DataflowFunctionMT, DataflowQueueMP, DataflowFunctionMP
from vsp.utils import compose
from vsp.view import KeypointView
from vsp.tracker import NearestNeighbourTracker
from vsp.encoder import KeypointEncoder

import sklearn
from sklearn.ensemble import BaggingClassifier, RandomForestClassifier


class CaptureTimestamp(V4l2VideoCamera):

    def __init__(self, *args, **kwargs):
        if 'cut' in kwargs:
            self.cut = kwargs.pop('cut')
        else:
            self.cut = None
        super().__init__(*args, **kwargs)
        self.timestamps = []

    def read(self):
        frame = super().read()
        self.timestamps.append(time.time())
        if self.cut is not None:
            frame[:,:self.cut[0],:] = 0
            frame[:,-self.cut[1]:,:] = 0
        return frame


class CaptureTimestampCv(CvVideoCamera):

    def __init__(self, *args, **kwargs):
        if 'cut' in kwargs:
            self.cut = kwargs.pop('cut')
        else:
            self.cut = None
        super().__init__(*args, **kwargs)
        self.timestamps = []

    def read(self):
        frame = super().read()
        self.timestamps.append(time.time())
        if self.cut is not None:
            frame[:,:self.cut[0],:] = 0
            frame[:,-self.cut[1]:,:] = 0
        return frame

class TestProcessor(CameraStreamProcessorMT):
    def __init__(self, camera, view=None, display=None, writer=None, detector=None, tracker=None, encoder=None, win_size=4, v_thr=0.15):
        super().__init__(camera)
        self.camera = camera

    def process(self, num_frames, outfile=None, record=True):
        self.camera.timestamps = []
        kpts = super().process(num_frames, outfile)
        ts = self.camera.timestamps
        ts = np.array(ts)
        return kpts, ts

class CameraStreamProcessorSlip(CameraStreamProcessorMT):
    """Camera stream processor.
    """
    def __init__(self, camera, view=None, display=None, writer=None, detector=None, tracker=None, encoder=None, model = None, n_pins=109, win_size=4, v_thr=0.15):
        super().__init__(camera)
        self.camera = camera
        self.display = display
        self.display_incipient = CvVideoDisplay(name='incipient_signal')
        self.blank_image = np.zeros((800,800,3), np.uint8)
        self.view = view
        self.writer = writer
        self.detector = detector
        self.tracker = tracker
        self.encoder = encoder
        self.frame = None
        self.kpts = None
        self.n_pins = n_pins
        self.starting_pins_detected = False
        self.start = None
        self.prev = None
        self.win_size = win_size
        self.window = deque([])
        self.v_thr = v_thr
        self.ts = []
        self.save_kpts = np.ndarray((2000,37,2))
        self.f=0
        self.max_incipients = 0
        self._cancel = False
        self.clf = pkl.load(model)
     
    def detect_start_pins(self, num_frames):
        
        for i in range(num_frames):  
            
            frame = self.camera.read()
            kpts, frame = self.detector.detect(frame)
            
            if len(kpts) !=self.n_pins:
                self.detector.set_circle(circle_mask=(307,257,115))
                i = i-1
                continue
            elif len(kpts) == self.n_pins:
                self.starting_pins_detected = True
                self.start = [p.point for p in kpts]
                self.detector.set_circle(circle_mask=(307,257,150))
                print('Detected starting pins')
            
            if self.display:
                imkpt = self.view(frame, kpts)
                self.display.write(imkpt)
        
    def process(self, num_frames, record=True, outfile=None):
        # Connect writer thread to camera and start running, if required
        writer_f = None
        outfile = None
        self.writer = None

        # Run pipeline
        self._cancel = False
        incipients = []
        statics = list(range(self.n_pins))

        for i in range(num_frames):
            if self._cancel:
                break
            frame = self.camera.read()
            if not record:
                continue
            
            # detect keypoints in frame                
            if self.kpts is not None:
                self.prev = self.kpts   
                
            kpts, frame = self.detector.detect(frame)
            
            # if self.display:
            #     imkpt = self.view(frame, kpts)
            #     self.display.write(imkpt)

            if not self.starting_pins_detected and len(kpts) !=self.n_pins:
                self.detector.set_circle(circle_mask=(307,257,115))
                i = i-1
                continue
            elif not self.starting_pins_detected and len(kpts) == self.n_pins:
                self.starting_pins_detected = True
                self.start = [p.point for p in kpts]
                self.detector.set_circle(circle_mask=(307,257,150))
                print('Detected starting pins')
            
            kpts = self.tracker(kpts)
                         
            if self.display:
                imkpt = self.view(frame, kpts)
                self.display.write(imkpt)

            self.kpts = [p.point for p in kpts]
            displacements = self.calc_displacements()
            velocities = self.calc_velocities()
            
            slip_pred = 0
            if velocities is not None:
                data = np.hstack((displacements,velocities))
                data = data.reshape(1,-1)
                slip_pred = self.clf.predict(data)
                print(slip_pred)
            
            
            if(slip_pred==0):
                self.blank_image[:]=(0,255,0)
            elif(slip_pred==1):
                self.blank_image[:]=(0,154,255)
            elif(slip_pred==2):
                self.blank_image[:]=(0,0,255)
            else:
                self.blank_image[:]=(0,255,0)
            
            self.display_incipient.write(self.blank_image)


            # for q in self.camera_out_q:
            #     q.put(frame)
            # if self.max_incipients >= 4:
            #     print(i,'INCIPIENT')
            #     break

        # # Flush pipeline
        # if len(self.pipeline) > 0:
        #     for q in self.camera_out_q:
        #         q.join()
        # else:
        #     for q in self.camera_out_q[1:]:
        #         q.join()

        # # Stop writer thread and disconnect from pipeline
        # if writer_f:
        #     writer_f.in_queues[0].close()
        #     writer_f.join()
        #     self.camera_out_q.pop()

        # # Get results from pipeline (or camera if no pipeline specified)
        # results = []
        # if len(self.pipeline) > 0:
        #     while self.pipeline_out_q[0].qsize() > 0:
        #         results.append(self.pipeline_out_q[0].get())
        # else:
        #     while self.camera_out_q[0].qsize() > 0:
        #         results.append(self.camera_out_q[0].get())

        return self.max_incipients

    def calc_displacements(self):
            
        if self.start is None:
            return [0]*self.n_pins
        else:
            diff = [[self.kpts[i][0]-self.start[i][0],self.kpts[i][1]-self.start[i][1]] for i in range(len(self.kpts))]
            dist = [np.sqrt(diff[i][0]**2+diff[i][1]**2) for i in range(len(diff))]
            return dist
            
    def calc_velocities(self):
        if self.prev is None:
            return None
        else:
            diff = [[self.kpts[i][0]-self.prev[i][0],self.kpts[i][1]-self.prev[i][1]] for i in range(len(self.kpts))]
            dist = [np.sqrt(diff[i][0]**2+diff[i][1]**2) for i in range(len(diff))]
            return dist

    def reset_vars(self):
        self.tracker.keypoints = None
        self.kpts = None
        self.window = deque([])
        self.f = 0
        self.ts = []
        self.max_incipients = 0

    def cancel(self):
        self._cancel = True

    def close(self):           
        self.camera.close()


class CameraStreamProcessorIS(CameraStreamProcessorMT):
    def __init__(self, camera, view=None, display=None, writer=None, detector=None, tracker=None, encoder=None, win_size=4, v_thr=0.15):
        super().__init__(camera)
        self.camera = camera
        self.display = display
        self.view = view
        self.writer = writer
        self.detector = detector
        self.tracker = tracker
        self.encoder = encoder
        self.frame = None
        self.kpts = None
        self.prev = None
        self.win_size = win_size
        self.window = deque([])
        self.v_thr = v_thr
        self.save_kpts = np.ndarray((2000,37,2))
        self.f = 0

        if self.display:
            self.display_func = self.display.write
            self.display.open()

    def process(self, num_frames=10000):
        
        writer_f = None
        outfile = None
        self.writer = None
        self._cancel = False
        self.f = 0
        for i in range(num_frames):
            if self._cancel:
                break
            frame = self.camera.read()
            if self.kpts is not None:
                self.prev = self.kpts   
            kpts = self.detector.detect(frame)
            if self.kpts is None and len(kpts) != 37:
                continue
            self.kpts = self.tracker(kpts)
            self.kpts = [p.point for p in self.kpts]
            self.save_kpts[self.f] = self.kpts
            self.f += 1
            if self.display:
                imkpt = self.view(frame, kpts)
                self.display.write(imkpt)

            for q in self.camera_out_q:
                q.put(frame)

        # Flush pipeline
        if len(self.pipeline) > 0:
            for q in self.camera_out_q:
                q.join()
        else:
            for q in self.camera_out_q[1:]:
                q.join()

        # Stop writer thread and disconnect from pipeline
        if writer_f:
            writer_f.in_queues[0].close()
            writer_f.join()
            self.camera_out_q.pop()

        # Get results from pipeline (or camera if no pipeline specified)
        results = []
        if len(self.pipeline) > 0:
            while self.pipeline_out_q[0].qsize() > 0:
                results.append(self.pipeline_out_q[0].get())
        else:
            while self.camera_out_q[0].qsize() > 0:
                results.append(self.camera_out_q[0].get())

        return self.kpts

    def calc_v_rels(self, kpts, s, i):
        if self.prev is None:
            return s,i
        else:
            diff = [[self.kpts[i][0]-self.prev[i][0],self.kpts[i][1]-self.prev[i][1]] for i in range(len(self.kpts))]
            if len(self.window) == self.win_size:
                self.window.popleft()
            self.window.append(diff)
            
            if len(self.window) == self.win_size:
                mov_avg = np.mean(self.window, axis=0)
                static_mean = [np.mean(mov_avg[s,0]),np.mean(mov_avg[s,1])]

                for p in s:
                    if np.hypot(mov_avg[p,0]- static_mean[0], mov_avg[p,1]- static_mean[1]) > self.v_thr:
                        s.remove(p)
                        i.append(p)
                for p in i:
                    if np.hypot(mov_avg[p,0]- static_mean[0], mov_avg[p,1]- static_mean[1]) <= self.v_thr:
                        s.append(p)
                        i.remove(p)

            return s, i
        
    def reset_vars(self):
        self.tracker.keypoints = None
        self.kpts = None
        self.window = deque([])
        self.camera.timestamps = []
        self.f = 0

    def cancel(self):
        self._cancel = True

    def close(self):           
        self.camera.close()

        
class CameraStreamProcessorMTTimestamp(CameraStreamProcessorMT):
    
    def __init__(self, camera, *args, **kwargs):
        super().__init__(camera=camera, *args,**kwargs)
        self.camera = camera

    def process(self, num_frames, outfile=None):
        self.camera.timestamps = []
        kpts = super().process(num_frames, outfile)
        ts = self.camera.timestamps
        ts = np.array(ts)
        return kpts, ts

class CameraStreamProcessorMPTimestamp(CameraStreamProcessorMP):
    
    def __init__(self, camera, *args, **kwargs):
        super().__init__(camera=camera, *args,**kwargs)
        self.camera = camera

    def process(self, num_frames, outfile=None):
        self.camera.timestamps = []
        kpts = super().process(num_frames, outfile)
        ts = self.camera.timestamps
        ts = np.array(ts)
        return kpts, ts

    def reset_vars(self):
        self.tracker.keypoints = None
        self.kpts = None
        self.window = deque([])
        self.camera.timestamps = []
        self.f = 0

class CameraStreamProcessorMPOnline(CameraStreamProcessorMP):
    
    def __init__(self, camera, *args, **kwargs):
        super().__init__(camera=camera, *args,**kwargs)
        self.camera = camera

    def process(self, num_frames, outfile=None):
        self.camera.timestamps = []
        kpts = super().process(num_frames, outfile)
        ts = self.camera.timestamps
        ts = np.array(ts)
        return kpts, ts

    def reset_vars(self):
        self.tracker.keypoints = None
        self.kpts = None
        self.window = deque([])
        self.camera.timestamps = []
        self.f = 0


class CaptureTimestampOnline(V4l2VideoCamera):

    def __init__(self, *args, **kwargs):
        if 'cut' in kwargs:
            self.cut = kwargs.pop('cut')
        else:
            self.cut = None
        super().__init__(*args, **kwargs)
        self.res = (640, 480)
        self.x_cut = (20, 65)
        self.ds = 0.1
        self.final_shape = (int(self.ds * (self.res[0] - sum(self.x_cut))), int(self.ds * self.res[1]))

    def read(self):
        frame = super().read()
        frame = process_image(frame, gray=True, bbox=(self.x_cut[0], 0, self.res[0] - self.x_cut[1], self.res[1]), dims=self.final_shape,
                                  threshold=[11, 2])
        return frame



class OnlineProcessingMT(CameraStreamProcessorMT):

    def __init__(self, camera, *args, **kwargs):
        super().__init__(camera=camera, *args,**kwargs)
        # self.camera = camera
        # self.model = tf.keras.models.load_model('is_10stack_aug.h5')
        # print(self.model.summary())

    def process(self, num_frames, outfile=None):
        i = 0

        while i < num_frames:
            frames = super().process(7)
            print(np.argmax(np.array(self.model.predict(np.expand_dims(frames, axis=0)))))
            i += 7


class CameraStreamProcessorMTCNN(CameraStreamProcessorMT):
    """Camera stream processor.
    """
    def __init__(self, camera, *args, **kwargs):
        super().__init__(camera, *args, **kwargs)
        self.model_name = 'is_10stack_aug.h5'
        self.model = tf.keras.models.load_model('is_10stack_aug.h5')
        print(self.model.summary())

    def process(self, num_frames, outfile=None):
        # Connect writer thread to camera and start running, if required
        writer_f = None
        outfile = None
        self.writer = None
        if outfile and self.writer:
            self.writer.filename = outfile
            self.camera_out_q.append(DataflowQueueMT(self.qsize))
            self.writer_in_q = [self.camera_out_q[-1]]
            writer_f = DataflowFunctionMT(func=self.writer.write,
                                      pre_func=self.writer.open,
                                      post_func=self.writer.close,
                                      in_queues=self.writer_in_q,
                                      ) 
            writer_f.start()

        # Run pipeline
        self._cancel = False
        n_stack = 10
        if self.model_name[-4] == 'g':
            model_array = np.zeros((1, self.camera.final_shape[1], self.camera.final_shape[0], 1))
        else:
            model_array = np.ndarray((1, n_stack, self.camera.final_shape[1], self.camera.final_shape[0], 1))
        
        result_sum = 0
        n_tests = 0
        for i in range(num_frames):
            if i % n_stack == 0:
                j = 0
                if i > 0:
                    inference = np.argmax(self.model.predict(model_array))
                    result_sum += inference
                    n_tests += 1
            if self._cancel:
                break
            frame = self.camera.read()

            if self.model_name[-4] == 'g':
                model_array[0, :, :, :] += frame/n_stack
            else:
                model_array[0, j] = frame
            j += 1
            for q in self.camera_out_q:
                q.put(frame)

        # Flush pipeline
        if len(self.pipeline) > 0:
            for q in self.camera_out_q:
                q.join()
        else:
            for q in self.camera_out_q[1:]:
                q.join()

        # Stop writer thread and disconnect from pipeline
        if writer_f:
            writer_f.in_queues[0].close()
            writer_f.join()
            self.camera_out_q.pop()

        # Get results from pipeline (or camera if no pipeline specified)
        results = []
        if len(self.pipeline) > 0:
            while self.pipeline_out_q[0].qsize() > 0:
                results.append(self.pipeline_out_q[0].get())
        else:
            while self.camera_out_q[0].qsize() > 0:
                results.append(self.camera_out_q[0].get())

        return result_sum, n_tests


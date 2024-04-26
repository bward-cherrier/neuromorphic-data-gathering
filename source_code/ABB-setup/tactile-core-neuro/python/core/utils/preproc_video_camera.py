# -*- coding: utf-8 -*-

from vsp.video_stream import CvVideoCamera
import cv2
import numpy as np

# imports for pre-processing code

class CvPreprocVideoCamera(CvVideoCamera):
    def __init__(self, 
                 size = None,
                 crop = None,
                 threshold = None, 
                 *args, **kwargs):  
        self.size = size
        self.crop = crop
        self.threshold = threshold
        super().__init__(*args, **kwargs)

    def __getstate__(self):   # needed for correct object serialization
        return super().__getstate__(), self.__dict__.copy()

    def __setstate__(self, state):   # needed for correct object serialization
        super().__setstate__(state[0])
        self.__dict__.update(state[1])

    def __call__(self):
        return self.read()

    def read(self):
        frame = super().read()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.crop is not None:
            x0, y0, x1, y1 = self.crop
            frame = frame[y0:y1, x0:x1]
        if self.threshold is not None:
            frame = frame.astype('uint8')
            frame = cv2.medianBlur(frame, 5)
            width, offset = self.threshold
            frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,                                       
                                          cv2.THRESH_BINARY, width, offset)
        if self.size is not None:
            frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        frame = frame[..., np.newaxis]  
        return frame

def main():
    size = (160, 120)
    crop = None
    threshold = (11, 0)
    ppcam = CvPreprocVideoCamera(size, crop, threshold)
    frame = ppcam()
    print(frame.shape)

if __name__ == '__main__':
    main()

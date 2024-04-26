# -*- coding: utf-8 -*-

import io, os
import matlab.engine
import numpy as np


out = None
err = None
eng = None

class ControlMatlab:
    def __init__(self):
        global out, err, eng
        
        if eng is None:
            eng = matlab.engine.start_matlab()
            eng.addpath(eng.genpath(os.environ['PYTHONPATH']+r'\..\matlab\experiments\commonScripts'))
        if out is None:
            out = io.StringIO()
        if err is None:
            err = io.StringIO()                              

    def transFrame(self, vec, frame):
        future = eng.transFrame(vec.tolist(), frame.tolist(), 
                                 stdout=out, stderr=err, background=True)
  
        vec_frame = future.result()
        print(out.getvalue(), err.getvalue())
        
        return np.asarray(vec_frame) 
    
def main():
    pass
    
if __name__ == '__main__':
    main()

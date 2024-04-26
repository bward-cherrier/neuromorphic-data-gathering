# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

import os, io
import matlab.engine

out = None
err = None
eng = None

class PlotContour:
    def __init__(self, poses=None, 
                 path=os.path.join(os.environ['PYTHONPATH'],'core','utils')):

        global out, err, eng
        
        if eng is None:
            eng = matlab.engine.start_matlab()
            eng.addpath(eng.genpath(path))      
        if out is None:
            out = io.StringIO()
        if err is None:
            err = io.StringIO()  
            
        poses = matlab.double(poses.tolist())            
        self._plot = eng.PlotContour(poses)           
        
    def update(self, u):
        u = matlab.double(u.tolist())   
        
        future = eng.update(self._plot, u, stdout=out, stderr=err, background=True, nargout=0)        
        future.result()
        print(out.getvalue(), err.getvalue())
        
    def finish(self, filename):
        future = eng.finish(self._plot, filename, stdout=out, stderr=err, background=True, nargout=0)        
        future.result()
        print(out.getvalue(), err.getvalue())
        
def main():
    None

if __name__ == '__main__':
    main()

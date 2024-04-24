# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

import os, io
import matlab.engine

out = None
err = None
eng = None

class PlotControl:
    def __init__(self, inds=[2,6],
                 path=os.path.join(os.environ['PYTHONPATH'],'core','utils')):

        global out, err, eng
        
        if eng is None:
            eng = matlab.engine.start_matlab()
            eng.addpath(eng.genpath(path))      
        if out is None:
            out = io.StringIO()
        if err is None:
            err = io.StringIO()  
            
        inds = matlab.double([i+1 for i in inds])            
        self._plot = eng.PlotControl(inds)           
        
    def update(self, i, y, e, ei):
        y = matlab.double(y.tolist())   
        e = matlab.double(e.tolist())   
        ei = matlab.double(ei.tolist())   
        
        future = eng.update(self._plot, i+1, y, e, ei, stdout=out, stderr=err, background=True, nargout=0)        
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

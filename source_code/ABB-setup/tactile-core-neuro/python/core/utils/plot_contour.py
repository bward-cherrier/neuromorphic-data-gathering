# -*- coding: utf-8 -*-

import matplotlib
import numpy as np

matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from core.utils.set_axes_equal import set_axes_equal
from win32api import GetSystemMetrics


class PlotContour:
    def __init__(self, poses):
#        plt.ion()
        self._fig = plt.figure(figsize=(8, 5), num=2)
        self._ax = self._fig.add_subplot(111, projection='3d')
        self._ax.plot([], [], [], '-r')  
        try:
            self._ax.plot(poses[:,1], poses[:,0], -poses[:,2], ':g')  
        except:
            pass
        
        geom = self._fig.canvas.manager.window.geometry()      
        x,y,dx,dy = geom.getRect()
        sx,sy = GetSystemMetrics(0), GetSystemMetrics(1)
        self._fig.canvas.manager.window.setGeometry(sx-dx, 0, dx, dy)
        plt.draw()
        
    def update(self, u):
        xdata, ydata, zdata = self._ax.lines[0]._verts3d
        xd = np.append(xdata, u[1]) # note flip x and y axes
        yd = np.append(ydata, u[0])
        zd = np.append(zdata, -u[2]) # flip z axis
        self._ax.set(xlim=[np.min(xd)-0.5,np.max(xd)+0.5], ylim=[np.min(yd)-0.5,np.max(yd)+0.5])
        self._ax.lines[0].set(xdata=xd, ydata=yd)
        self._ax.lines[0].set_3d_properties(zd)
        self._ax.set(zlim=[np.min(zd)-0.5, np.max(zd)+0.5])
        set_axes_equal(self._ax)
        plt.draw()
        plt.pause(0.001)

    def finish(self, filename):
        self._fig.savefig(filename, bbox_inches='tight')   
#        plt.show(block=True)
        
        
def main():
    None

if __name__ == '__main__':
    main()

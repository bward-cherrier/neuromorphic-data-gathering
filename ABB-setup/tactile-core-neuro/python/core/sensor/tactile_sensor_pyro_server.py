# -*- coding: utf-8 -*-
"""
Created on Tue Nov  6 09:58:56 2018

@author: John
"""

from __future__ import division, print_function, unicode_literals

import argparse
import Pyro4

from core.sensor.tactile_sensor import TactileSensor
from core.utils.start_pyro_server import start_pyro_server


class TactileSensorServer:
    def __init__(self):
        self._sensor = None
        self._result = None
        self._cancel = False
        
    def _is_cancel(self):
        return self._cancel
    
    def init(self, *args, **kwargs):
        self._sensor = TactileSensor(*args, **kwargs)
        
    def init_pins(self, max_pin_dist_from_centroid=300, min_pin_separation=0, filename=None):
        return self._sensor.init_pins(max_pin_dist_from_centroid, min_pin_separation, filename)

    def set_pins(self, pins):
        self._sensor.set_pins(pins)
        
    def record_pins(self, num_samples=1000000, filename=None, cancel=None):
        return self._sensor.record_pins(num_samples, filename, cancel)
        
    def record_frames(self, num_samples=1000000, filename=None, cancel=None):
        return self._sensor.record_frames(num_samples, filename, cancel)
    
    def replay_frames(self, filename):
        return self._sensor.replay_frames(filename)
    
    def replay_pins(self, filename):
        return self._sensor.replay_pins(filename)    

    def record_pins_with_time_stamp(self, num_samples=1000000, filename=None, cancel=None):
        return self._sensor.record_pins_with_time_stamp(num_samples, filename, cancel)
        
    def async_track_pins(self, filename=None):
        self._cancel = False
        func = Pyro4.Future(self._sensor.track_pins)
        self._result = func(filename, cancel=self._is_cancel)
        
    def async_record_pins(self, num_samples=1000000, filename=None, cancel=None):
        self._cancel = False
        func = Pyro4.Future(self._sensor.record_pins)
        self._result = func(num_samples, filename, cancel=self._is_cancel)
        
    def async_record_frames(self, num_samples=1000000, filename=None, cancel=None):
        self._cancel = False
        func = Pyro4.Future(self._sensor.record_frames)
        self._result = func(num_samples, filename, cancel=self._is_cancel)
        
    def async_cancel(self):
        self._cancel = True
        
    def async_result(self):
        return self._result.value
    

def main():
    parser = argparse.ArgumentParser(
            description='Instantiate object and register with name server')
    parser.add_argument('-o', '--object_id', required=True, dest='object_id',
                        action='store', help='ID of object to instantiate and register')    
    args = parser.parse_args()
    
    start_pyro_server(object_class=TactileSensorServer, object_id=args.object_id, expose_class=True)

    
if __name__ == '__main__':
    main()
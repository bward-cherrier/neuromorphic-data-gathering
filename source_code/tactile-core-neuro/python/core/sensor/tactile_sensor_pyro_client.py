# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 12:25:13 2018

@author: John
"""

import argparse
import Pyro4


class TactileSensorClient:

    def __init__(self, object_id):
        Pyro4.config.SERIALIZER = 'pickle'      
        self._proxy = Pyro4.Proxy('PYRONAME:' + object_id)

    def init(self, *args, **kwargs):
        self._proxy.init(*args, **kwargs)

    def init_pins(self, max_pin_dist_from_centroid=300, min_pin_separation=0, filename=None):
        return self._proxy.init_pins(max_pin_dist_from_centroid, min_pin_separation, filename)

    def set_pins(self, pins):
        self._proxy.set_pins(pins)
        
    def record_pins(self, num_samples=1000000, filename=None, cancel=None):
        return self._proxy.record_pins(num_samples, filename, cancel)
        
    def record_frames(self, num_samples=1000000, filename=None, cancel=None):
        return self._proxy.record_frames(num_samples, filename, cancel)
    
    def replay_frames(self, filename):
        return self._proxy.replay_frames(filename)
    
    def replay_pins(self, filename):
        return self._proxy.replay_pins(filename)    

    def record_pins_with_time_stamp(self, num_samples=1000000, filename=None, cancel=None):
        return self._proxy.record_pins_with_time_stamp(num_samples, filename, cancel)
        
    def async_track_pins(self, filename=None):
        self._cancel = False
        func = Pyro4.Future(self._proxy.track_pins)
        self._result = func(filename, cancel=self._is_cancel)
        
    def async_record_pins(self, num_samples=1000000, filename=None, cancel=None):
        self._cancel = False
        func = Pyro4.Future(self._proxy.record_pins)
        self._result = func(num_samples, filename, cancel=self._is_cancel)
        
    def async_record_frames(self, num_samples=1000000, filename=None, cancel=None):
        self._cancel = False
        func = Pyro4.Future(self._proxy.record_frames)
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

    print('Initialising tactile sensor ...')
    sensor = TactileSensorClient(args.object_id)
    sensor.init(video_source=0,
                brightness=150,
                contrast=10,
                saturation=0,
                exposure=-6) 
    pins = sensor.record_pins(100)
    print(pins.shape)


if __name__ == '__main__':
    main()
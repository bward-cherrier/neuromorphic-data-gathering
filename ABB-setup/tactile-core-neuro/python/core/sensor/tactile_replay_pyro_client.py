# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 12:25:13 2018

@author: John
"""

import argparse
import Pyro4


class TactileReplayClient:

    def __init__(self, object_id):
        Pyro4.config.SERIALIZER = 'pickle'      
        self._proxy = Pyro4.Proxy('PYRONAME:' + object_id)

    def init(self, *args, **kwargs):
        self._proxy.init(*args, **kwargs)

    def init_pins(self, filename, max_pin_dist_from_centroid=300, min_pin_separation=0):
        return self._proxy.init_pins(filename, max_pin_dist_from_centroid, min_pin_separation)

    def set_pins(self, pins):
        self._proxy.set_pins(pins)

    def replay_pins(self, filename):
        return self._proxy.replay_pins(filename) 
    
    def replay_frames(self, filename):
        return self._proxy.replay_frames(filename)


def main():
    parser = argparse.ArgumentParser(
            description='Instantiate object and register with name server')
    parser.add_argument('-o', '--object_id', required=True, dest='object_id',
                        action='store', help='ID of object to instantiate and register')    
    args = parser.parse_args()

    print('Initialising tactile replay ...')
    sensor = TactileReplayClient(args.object_id)
    sensor.init()


if __name__ == '__main__':
    main()

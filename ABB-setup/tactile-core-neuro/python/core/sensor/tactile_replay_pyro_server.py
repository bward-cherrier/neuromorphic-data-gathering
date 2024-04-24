# -*- coding: utf-8 -*-
"""
Created on Tue Nov  6 09:58:56 2018

@author: John
"""

from __future__ import division, print_function, unicode_literals

import argparse

from core.sensor.tactile_replay import TactileReplay
from core.utils.start_pyro_server import start_pyro_server


class TactileReplayServer:
    def __init__(self):
        self._replay = None
    
    def init(self, *args, **kwargs):
        self._replay = TactileReplay(*args, **kwargs)

    def init_pins(self, filename, max_pin_dist_from_centroid=300, min_pin_separation=0):
        return self._replay.init_pins(filename, max_pin_dist_from_centroid, min_pin_separation)

    def set_pins(self, pins):
        self._replay.set_pins(pins)

    def replay_pins(self, filename):
        return self._replay.replay_pins(filename)   
    
    def replay_frames(self, filename):
        return self._replay.replay_frames(filename)
    

def main():
    parser = argparse.ArgumentParser(
            description='Instantiate object and register with name server')
    parser.add_argument('-o', '--object_id', required=True, dest='object_id',
                        action='store', help='ID of object to instantiate and register')    
    args = parser.parse_args()
    
    start_pyro_server(object_class=TactileReplayServer, object_id=args.object_id, expose_class=True)

    
if __name__ == '__main__':
    main()
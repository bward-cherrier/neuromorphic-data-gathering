# -*- coding: utf-8 -*-
"""
Created on Sat Mar 25 17:15:45 2017

@author: John Lloyd

This module is can be used to create an RPC client for a TactileSensor object
running in a separate process.  It is based on the RPCAutoProxy class in the
rpc module.

Some possible usage patterns are as follows:
    
import time
    
sensor = TactileSensorClient(mask_type='circle',
                             mask_centre=(310, 230),
                             mask_radius=220)

pins = sensor.init_pins()
# sensor.set_pins(pins)

sensor.async_track_pins()  # non-blocking
time.sleep(5)
sensor.async_cancel()  # non-blocking
sensor.async_result()  # blocking

sensor.async_record_pins()  # non-blocking
time.sleep(1)
sensor.async_cancel()  # non-blocking
data = sensor.async_result()   # blocking

data = sensor.record_pins(10)  #blocking    

"""

import os
import sys
import argparse
import time

from core.utils.rpc import RPCAutoProxy


class TactileSensorClient(RPCAutoProxy):
    def __init__(self, *args, **kwargs):
        script_dir = os.path.dirname(__file__)
        script_name = 'tactile_sensor_server.py'
        script_path = os.path.join(script_dir, script_name)
        RPCAutoProxy.__init__(self, script_path, 'localhost', 17000)
        time.sleep(1)
        self.register_instance(*args, **kwargs)


def main(address, port):
    
#    sensor = TactileSensorProxy(pin_tracking=False)
#    try:
#        frames = sensor.record_frames(100)
#        print(frames.shape)
#    finally:
#        del sensor

    sensor = TactileSensorClient()
    try:
        print('Calling async_track_pins()')
        t0 = time.clock()
        sensor.async_track_pins()
        t1 = time.clock()
        print('Elapsed time = ', t1 - t0)
        time.sleep(10)
        print('Calling async_cancel()')
        t2 = time.clock()
        sensor.async_cancel()
        t3 = time.clock()
        print('Elapsed time = ', t3 - t2)
        print('Calling async_result()')
        t4 = time.clock()    
        sensor.async_result()
        t5 = time.clock()
        print('Elapsed time = ', t5 - t4)
        print('Overall time = ', t5 - t0)
        sys.stdout.flush()
        
        print('Calling async_record_pins()')
        t0 = time.clock()
        sensor.async_record_pins()
        t1 = time.clock()
        print('Elapsed time = ', t1 - t0)
        time.sleep(1)
        print('Calling async_cancel()')
        t2 = time.clock()
        sensor.async_cancel()
        t3 = time.clock()
        print('Elapsed time = ', t3 - t2)
        print('Calling async_result()')
        t4 = time.clock()    
        pins = sensor.async_result()
        t5 = time.clock()
        print('Elapsed time = ', t5 - t4)
        print('Record time = ', t3 - t0)
        print('Overall time = ', t5 - t0)
        print(pins)
    finally:
        del sensor

if __name__ == '__main__': 
    parser = argparse.ArgumentParser(description='Start object server')
    parser.add_argument('-a', '--address', dest='address', default='localhost', help='Server IP address')
    parser.add_argument('-p', '--port', dest='port', default='17000', type=int, help='Server port')
    args = parser.parse_args()
    
    main(args.address, args.port)

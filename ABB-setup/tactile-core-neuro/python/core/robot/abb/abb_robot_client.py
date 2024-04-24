# -*- coding: utf-8 -*-
"""
Created on Sun Mar 26 16:54:21 2017

@author: John Lloyd

This module is can be used to create an RPC client for an ABBProxy object
running in a separate process.  It is based on the RPCAutoProxy class in the
rpc module.

Some possible usage patterns are as follows:
    
1. Set up an interface with an ABB controller server listening on port 5000
at ip address 164.11.72.43, adjust some settings and carry out an asynchronous
linear move in Cartesian space.

robot = ABBRobotClient('164.11.72.43', 5000)   
robot.set_tool([[0,0,89.1], [1,0,0,0]])
robot.set_speed([20, 10, 20, 10])
robot.async_set_cartesian([[400,0,300], [0,0,-1,0]])
robot.async_result()

"""

import os
import argparse
import time

from core.utils.rpc import RPCAutoProxy

class ABBRobotClient(RPCAutoProxy):
    def __init__(self, *args, **kwargs):
        script_dir = os.path.dirname(__file__)
        script_name = 'abb_robot_server.py'
        script_path = os.path.join(script_dir, script_name)                
        RPCAutoProxy.__init__(self, script_path, 'localhost', 18000)
        time.sleep(1)
        self.register_instance(*args, **kwargs)

def main(address, port):
    robot = ABBRobotClient('164.11.72.68')  
    robot.set_speed([20, 10, 20, 10])
    robot.set_tool([[0,0,89.1],[1,0,0,0]])
    robot.set_zone('z0', True)
    robot.async_set_cartesian([[400,0,300], [0,0,-1,0]])
    robot.async_result()
    del robot

if __name__ == '__main__': 
    parser = argparse.ArgumentParser(description='Start object server')
    parser.add_argument('-a', '--address', dest='address', default='localhost', help='Server IP address')
    parser.add_argument('-p', '--port', dest='port', default='17000', type=int, help='Server port')
    args = parser.parse_args()
    
    main(args.address, args.port)


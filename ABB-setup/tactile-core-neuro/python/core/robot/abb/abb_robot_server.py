# -*- coding: utf-8 -*-
"""
Created on Sun Mar 26 16:43:59 2017

@author: John Lloyd

This module is can be used to create an RPC server for an ABBProxy object
running in a separate process.  It is based on the RPCServer class in the
rpc module.

"""

import argparse

from core.utils.rpc import RPCServer
from abb_robot import ABBRobotProxy

def main(address, port):
    server = RPCServer(address, port)
    server.register_class(ABBRobotProxy)
    server.serve_forever()

if __name__ == '__main__':   
    parser = argparse.ArgumentParser(description='Start object server')
    parser.add_argument('-a', '--address', dest='address', default='localhost', help='Server IP address')
    parser.add_argument('-p', '--port', dest='port', default='17000', type=int, help='Server port')
    args = parser.parse_args()
    
    main(args.address, args.port)

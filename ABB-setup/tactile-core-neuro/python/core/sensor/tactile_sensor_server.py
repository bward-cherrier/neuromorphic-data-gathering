# -*- coding: utf-8 -*-
"""
Created on Fri Mar 24 14:19:39 2017

@author: John Lloyd

This module is can be used to create an RPC server for a TactileSensorProxy
object running in a separate process.  It is based on the RPCServer class in the
rpc module.

"""

import argparse

from core.utils.rpc import RPCServer
from tactile_sensor import TactileSensorProxy


def main(address, port):
    server = RPCServer(address, port)
    server.register_class(TactileSensorProxy)
    server.serve_forever()


if __name__ == '__main__':   
    parser = argparse.ArgumentParser(description='Start object server')
    parser.add_argument('-a', '--address', dest='address', default='localhost', help='Server IP address')
    parser.add_argument('-p', '--port', dest='port', default='17000', type=int, help='Server port')
    args = parser.parse_args()
    
    main(args.address, args.port)

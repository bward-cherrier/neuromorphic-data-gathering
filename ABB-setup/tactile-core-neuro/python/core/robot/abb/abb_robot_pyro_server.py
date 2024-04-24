# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 14:07:33 2018

@author: John
"""

from __future__ import division, print_function, unicode_literals

import argparse
import Pyro4

from core.robot.abb.abb_robot import ABBRobot
from core.utils.start_pyro_server import start_pyro_server


class ABBRobotServer:
    def __init__(self):
        self._robot = None
        self._result = None
    
    def init(self, *args, **kwargs):
        self._robot = ABBRobot(*args, **kwargs)
        
    def connect_motion(self, remote):          
        self._robot.connect_motion(remote)

    def connect_logger(self, remote, maxlen=None):
        self._robot.connect_logger(remote, maxlen)

    def get_robotinfo(self):
        return self._robot.get_robotinfo()

    def set_tool(self, tool=[[0,0,0], [1,0,0,0]]):
        self._robot.set_tool(tool)
        
    def get_tool(self):
        return self._robot.get_tool()
    
    def set_units(self, linear, angular):
        self._robot.set_units(linear, angular)

    def set_speed(self, speed=[100,50,50,50]):
        self._robot.set_speed(speed)

    def set_cartesian(self, pose):
        return self._robot.set_cartesian(pose)
    
    def get_cartesian(self):
        return self._robot.get_cartesian()

    def set_joints(self, joints):
        return self._robot.set_joints(joints)

    def get_joints(self):
        return self._robot.get_joints()
    
    def set_external_axis(self, axis_values=[-550,0,0,0,0,0]):
        return self._robot.set_external_axis(axis_values)
    
    def get_external_axis(self):
        return self._robot.get_external_axis()
    
    def set_workobject(self, work_obj=[[0,0,0],[1,0,0,0]]):
        self._robot.set_workobject(work_obj)

    def set_zone(self, 
                 zone_key     = 'z1', 
                 point_motion = False, 
                 manual_zone  = []):
        self._robot.set_zone(zone_key, point_motion, manual_zone)

    def buffer_add(self, pose):
        self._robot.buffer_add(pose)
        
    def buffer_set(self, pose_list):
        return self._robot.buffer_set(pose_list)
        
    def clear_buffer(self):
        return self._robot.clear_buffer()

    def buffer_len(self):
        return self._robot.buffer_len()
    
    def buffer_execute(self):
        return self._robot.buffer_execute()
    
    def move_circular(self, pose_onarc, pose_end):
        return self._robot.move_circular(pose_onarc, pose_end)

    def async_set_cartesian(self, pose):
        func = Pyro4.Future(self._robot.set_cartesian)
        self._result = func(pose)

    def async_set_joints(self, joints):
        func = Pyro4.Future(self._robot.set_joints)
        self._result = func(joints)
        
    def async_buffer_execute(self):
        func = Pyro4.Future(self._robot.buffer_execute)
        self._result = func()
        
    def async_move_circular(self, pose_onarc, pose_end):
        func = Pyro4.Future(self._robot.move_circular)
        self._result = func(pose_onarc, pose_end)
        
    def async_result(self):
        return self._result.value
    

def main():
    parser = argparse.ArgumentParser(
            description='Instantiate object and register with name server')
    parser.add_argument('-o', '--object_id', required=True, dest='object_id',
                        action='store', help='ID of object to instantiate and register')    
    args = parser.parse_args()
    
    start_pyro_server(object_class=ABBRobotServer, object_id=args.object_id, expose_class=True)

    
if __name__ == '__main__':
    main()

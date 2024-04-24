# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 16:07:27 2018

@author: jl15313
"""

import argparse
import Pyro4


class ABBRobotClient:

    def __init__(self, object_id):
        Pyro4.config.SERIALIZER = 'pickle'      
        self._proxy = Pyro4.Proxy('PYRONAME:' + object_id)

    def init(self, *args, **kwargs):
        self._proxy.init(*args, **kwargs)
        
    def connect_motion(self, remote):          
        self._proxy.connect_motion(remote)

    def connect_logger(self, remote, maxlen=None):
        self._proxy.connect_logger(remote, maxlen)

    def get_robotinfo(self):
        return self._proxy.get_robotinfo()

    def set_tool(self, tool=[[0,0,0], [1,0,0,0]]):
        self._proxy.set_tool(tool)
        
    def get_tool(self):
        return self._proxy.get_tool()
    
    def set_units(self, linear, angular):
        self._proxy.set_units(linear, angular)

    def set_speed(self, speed=[100,50,50,50]):
        self._proxy.set_speed(speed)

    def set_cartesian(self, pose):
        return self._proxy.set_cartesian(pose)
    
    def get_cartesian(self):
        return self._proxy.get_cartesian()

    def set_joints(self, joints):
        return self._proxy.set_joints(joints)

    def get_joints(self):
        return self._proxy.get_joints()
    
    def set_external_axis(self, axis_values=[-550,0,0,0,0,0]):
        return self._proxy.set_external_axis(axis_values)
    
    def get_external_axis(self):
        return self._proxy.get_external_axis()
    
    def set_workobject(self, work_obj=[[0,0,0],[1,0,0,0]]):
        self._proxy.set_workobject(work_obj)

    def set_zone(self, 
                 zone_key     = 'z1', 
                 point_motion = False, 
                 manual_zone  = []):
        self._proxy.set_zone(zone_key, point_motion, manual_zone)

    def buffer_add(self, pose):
        self._proxy.buffer_add(pose)
        
    def buffer_set(self, pose_list):
        return self._proxy.buffer_set(pose_list)
        
    def clear_buffer(self):
        return self._proxy.clear_buffer()

    def buffer_len(self):
        return self._proxy.buffer_len()
    
    def buffer_execute(self):
        return self._proxy.buffer_execute()
    
    def move_circular(self, pose_onarc, pose_end):
        return self._proxy.move_circular(pose_onarc, pose_end)

    def async_set_cartesian(self, pose):
        func = Pyro4.Future(self._proxy.set_cartesian)
        self._result = func(pose)

    def async_set_joints(self, joints):
        func = Pyro4.Future(self._proxy.set_joints)
        self._result = func(joints)
        
    def async_buffer_execute(self):
        func = Pyro4.Future(self._proxy.buffer_execute)
        self._result = func()
        
    def async_move_circular(self, pose_onarc, pose_end):
        func = Pyro4.Future(self._proxy.move_circular)
        self._result = func(pose_onarc, pose_end)
        
    def async_result(self):
        return self._result.value


def main():
    parser = argparse.ArgumentParser(
            description='Instantiate object and register with name server')
    parser.add_argument('-o', '--object_id', required=True, dest='object_id',
                        action='store', help='ID of object to instantiate and register')    
    args = parser.parse_args()

    print('Initialising robot ...')
    robot = ABBRobotClient(args.object_id)
    robot.init('164.11.72.127')
    robot.set_speed([20, 10, 20, 10])
    robot.set_tool([[0,0,89.1],[1,0,0,0]])
    robot.set_zone('z0', True)
    robot.set_cartesian([[400,0,300], [0,0,-1,0]])


if __name__ == '__main__':
    main()

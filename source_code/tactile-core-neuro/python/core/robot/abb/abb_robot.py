# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 13:51:09 2018

@author: John
"""

from abb import Robot


class ABBRobot(Robot):
    def __del__(self):
        self.close()

              
def main():
    robot = ABBRobot('164.11.72.127')
    robot.set_speed([20, 10, 20, 10])
    robot.set_tool([[0,0,89.1],[1,0,0,0]])
    robot.set_zone('z0', True)
    robot.set_cartesian([[400,0,300], [0,0,-1,0]])


if __name__ == '__main__':
    main()
    
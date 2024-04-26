#!/usr/bin/env python
import os, sys
import argparse
import time
from ur5 import UR5Robot
import numpy as np


print('starting')
robot = UR5Robot()
print('initialised robot')
robot.start()
print('completed start')



desired_positions = [[400, -33, 375, 180, -80, 0],
					 [430, -33, 375, 180, -80, 0],
					 [400, -33, 375, 180, -80, 0]]

response = robot.moveLinear(desired_positions[0])
print('1')

response = robot.moveLinear(desired_positions[1])
print('2')

response = robot.moveLinear(desired_positions[2])


print "shutting down robot client ..."

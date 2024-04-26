#!/usr/bin/env python

import time
from multiprocessing import Process, JoinableQueue, Condition
import socket
from math import radians
import copy
import numpy as np
import robot.ur.rtde.rtde as rtde
import robot.ur.rtde.rtde_config as rtde_config
import logging
import os, sys
import threading

import datetime


    ###############################################################################

class UR5Robot:
    def __init__(self):
        #sys.stdout = open('debug2.txt', 'a')
        #self.aa = euler2axisangle2.E2A()
        self.initPos = None
        self.con = None
        #self.c = None

    ###############################################################################

    def start(self):
        # change directory to allow for startup files to be imported
        full_path = os.path.realpath(__file__)
        newPath = os.path.dirname(full_path)
        oldPath = os.getcwd()
        os.chdir(newPath)

        # Initial log
        logging.getLogger().setLevel(logging.INFO)
        # SETUP TCP/IP CONNECTION WITH ROBOT CONTROLLER
        ROBOT_HOST = "192.11.72.10"  # The robot host IP
        ROBOT_PORT = 30004  # RTDE port - 125 Hz
        RTDE_PROTOCOL_VERSION = 1
        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()
        if not self.con.negotiate_protocol_version(RTDE_PROTOCOL_VERSION):
            sys.exit()
            print("protocol version not supported by controller!")

        # LOAD RTDE CONFIG FILE AND GET RECIPE DETAILS FOR DATA TRANSFER
        conf = rtde_config.ConfigFile("movel_loop_configuration.xml")
        state_names, state_types = conf.get_recipe('state')
        opcode_names, opcode_types = conf.get_recipe('opcode')
        setp_names, setp_types = conf.get_recipe('setp')
        setj_names, setj_types = conf.get_recipe('setj')
        setTCPspeed_names, setTCPspeed_types = conf.get_recipe('setTCPspeed')
        setJspeed_names, setJspeed_types = conf.get_recipe('setJspeed')
        watchdog_names, watchdog_types = conf.get_recipe('watchdog')

        # CONFIGURE INPUT/OUTPUT PACKAGES
        self.opcode = self.con.send_input_setup(opcode_names, opcode_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.setj = self.con.send_input_setup(setj_names, setj_types)
        self.setTCPspeed = self.con.send_input_setup(setTCPspeed_names, setTCPspeed_types)
        self.setJspeed = self.con.send_input_setup(setJspeed_names, setJspeed_types)
        self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
        self.con.send_output_setup(state_names, state_types)
        if not self.con.send_start():
            sys.exit()
        self.state = self.con.receive()
        self.watchdog.input_int_register_0 = 0
        self.opcode.input_int_register_1 = 0
        self.opcode.input_int_register_2 = 0
        self.con.send(self.opcode)

        param_file = open('exp_parameters.txt', 'r')
        exp_parameters = str(param_file.read()).split("|")
        self.initPos = [float(exp_parameters[0]), float(exp_parameters[1]),
                        float(exp_parameters[2]),
                        (float(exp_parameters[3])), (float(exp_parameters[4])), (float(exp_parameters[5]))]

        time.sleep(0.1)
        os.chdir(oldPath)
        self.e = threading.Event()
        self.thread_lock = threading.Lock()
        self.other_command = False
        self.thread_pause = False
        self.buffer_thread = threading.Thread(target=self.clear_buffer)
        self.buffer_thread.setDaemon(True)
        self.buffer_thread.start()
        self.speed = 5
        self.prev_status = 0

        print('Robot ready!')

    ###############################################################################

    def moveLinear(self, move, speed=None, block=True, interrupt=False):
        self.stop_thread()
        self.mv = move
        self.opcode.input_int_register_1 = 1 # Sets the op code
        self.opcode.input_int_register_2 = int(interrupt)
        self.setp.input_double_register_0 = move[0]*0.001
        self.setp.input_double_register_1 = move[1]*0.001
        self.setp.input_double_register_2 = move[2]*0.001
        self.setp.input_double_register_3 = radians(move[3]) # Send as RPY - UR5 handles conversion
        self.setp.input_double_register_4 = radians(move[4])
        self.setp.input_double_register_5 = radians(move[5])

        if speed is not None:
            self.setTCPspeed.input_double_register_12 = speed*0.001
            #self.speed = speed
        # else:
        #     self.setTCPspeed.input_double_register_12 = self.speed*0.001
        #     print(self.setTCPspeed.input_double_register_12)

        self.con.send(self.opcode)
        self.con.send(self.setp)
        self.con.send(self.setTCPspeed)
        if block:
            status = self.complete()
        else:
            status = 'Non-blocking move'
        self.restart_thread()
        return status

    ###############################################################################

    def moveJointSpace(self, move):
        self.stop_thread()
        self.opcode.input_int_register_1 = 2 # Sets the op code
        self.setj.input_double_register_6 = radians(move[0])
        self.setj.input_double_register_7 = radians(move[1])
        self.setj.input_double_register_8 = radians(move[2])
        self.setj.input_double_register_9 = radians(move[3])
        self.setj.input_double_register_10 = radians(move[4])
        self.setj.input_double_register_11 = radians(move[5])

        self.con.send(self.opcode)
        self.con.send(self.setj)
        status = self.complete()
        self.restart_thread()
        return status

    ###############################################################################

    def setTCPSpeed(self, speed):
        self.stop_thread()
        self.opcode.input_int_register_1 = 3 # Sets the op code
        self.setTCPspeed.input_double_register_12 = speed*0.001 # Give in mm/s - converts to m/s
        self.con.send(self.opcode)
        self.con.send(self.setTCPspeed)
        status = self.complete()
        self.restart_thread()
        #self.speed = speed
        return status

    ###############################################################################

    def setJointSpeed(self, speed):
        self.stop_thread()
        self.opcode.input_int_register_1 = 4 # Sets the op code
        self.setJspeed.input_double_register_13 = radians(speed) # Give in deg/s - converts to rad/s
        self.con.send(self.opcode)
        self.con.send(self.setJspeed)
        status = self.complete()
        self.restart_thread()
        return status

    ###############################################################################

    def getJoints(self):
        self.stop_thread()
        # Opcode = 6 (from MATLAB) - Doesn't need to be sent to robot as it publishes this info to registers without prompt
        self.opcode.input_int_register_1 = 0 # Set opcode to 0 so robot knows to do nothing!
        self.con.send(self.opcode)
        self.state = self.con.receive()
        jointPos = np.asarray(self.state.actual_q)
        jointPos = jointPos.astype(np.float32)
        self.restart_thread()
        return jointPos

    ###############################################################################

    def getPose(self):
        self.stop_thread()
        # Opcode = 5 (from MATLAB) - Doesn't need to be sent to robot as it publishes this info to registers without prompt
        self.opcode.input_int_register_1 = 0 # Set opcode to 0 so robot knows to do nothing!
        self.con.send(self.opcode)
        self.state = self.con.receive()
        tcpPose = np.asarray(self.state.actual_TCP_pose)
        tcpPose = tcpPose.astype(np.float32)
        self.restart_thread()
        return tcpPose


    ###############################################################################

    def complete(self):

        state = self.con.receive()
        status = state.output_int_register_0

        while status == 1 and self.prev_status == 1:
            self.prev_status = status
            state = self.con.receive()
            status = state.output_int_register_0

        while status == 0:
            state = self.con.receive()
            status = state.output_int_register_0

        self.prev_status = status

        if self.opcode.input_int_register_1 == 1 or self.opcode.input_int_register_1 == 2:
            response = "UR5 Move completed"
        elif self.opcode.input_int_register_1 == 3 or self.opcode.input_int_register_1 == 4:
            response = "UR5 Speed changed"

        print(response)

        self.opcode.input_int_register_1 = 0 # Set opcode to 0 so robot knows to do nothing!
        self.con.send(self.opcode)

        return response

    def sleep(self, timeout=10):
        print("UR5 sleeping for {} seconds".format(timeout))
        mustend = time.time() + timeout
        while time.time() < mustend:
            self.con.receive() # continuously read from buffer to avoid overflow
            time.sleep(0.01)

    ###############################################################################
    def clear_buffer(self):
        self.e.set()
        while self.e.isSet():
            self.thread_pause = False # Flag for whether thread has been paused
            self.con.receive()

            if self.other_command: # Flag to say 'pause thread'
                self.thread_pause = True   # lets other thread know thread paused
                self.thread_lock.acquire() # blocks thread until released

            time.sleep(0.001)

    ###############################################################################

    def is_complete(self):
        state = self.con.receive()
        status = state.output_int_register_0
        return status

    ###############################################################################

    def stop_thread(self):
        self.other_command = True
        while self.thread_pause == False:
            time.sleep(0.01)

    ###############################################################################

    def restart_thread(self):
        self.other_command = False
        # self.thread_lock.release()

    ###############################################################################

    def finish(self):
        self.e.clear()
        self.buffer_thread.join()
        self.con.send(self.watchdog)
        self.con.disconnect()
        print("shutting down client ...")

    ###############################################################################

if __name__ == "__main__":

    try:
        # INSTANTIATE EXPERIMENT OBJECT
        client = UR5Robot()
        client.start()
        #client.setTCPSpeed(0.005)
        #client.moveLinear([0.001, -0.400, 0.284, 0.1000000, 180, 0.1000000])
        #client.getJoints()
        #client.getPose()
        #client.moveJointSpace([90,-103.92,107.35,-93.51,-89.3,-15.93])
        #client.setJointSpeed(0.1)
        #client.moveJointSpace([74, -103.92, 107.35, -93.51, -89.3, -15.93])

    except Exception as e:
        print(e)
    finally:
        client.finish()

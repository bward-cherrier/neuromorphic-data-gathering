# -*- coding: utf-8 -*-

import unittest
import sys
import logging
import time
import numpy as np

from tactile.robot.abb.abb_robot_client import ABBRobotClient

class ABBRobotClientTestCase(unittest.TestCase):
    """
    Tests for ABBRobotClient in abb_robot_client.py
    """
    
    def assertJointsAlmostEqual(self, joints1, joints2, delta):
        for a, b in zip(joints1, joints2):
            self.assertAlmostEqual(a, b, delta=delta)       
    
    def assertPoseAlmostEqual(self, pose1, pose2, delta1, delta2):
        # Compare position
        for a, b in zip(pose1[0], pose2[0]):
            self.assertAlmostEqual(a, b, delta=delta1)
        # Compare orientation
        if all(np.sign(pose1[1]) == np.sign(pose2[1])):
            for a, b in zip(pose1[1], pose2[1]):
                self.assertAlmostEqual(a, b, delta=delta2)
        else:
            for a, b in zip(pose1[1], pose2[1]):
                self.assertAlmostEqual(a, -b, delta=delta2)            
    
    def setUp(self):
        """
        Initialise prior to each test
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.setUp')
        self._robot = ABBRobotClient('164.11.72.68')
        self._robot.set_speed([20, 10, 20, 10])
        self._robot.set_tool([[0, 0, 89.1], [1, 0, 0, 0]])
        self._robot.set_zone('z0', True)
        info =self._robot.get_robotinfo()
        logger.debug('Robot info = %s', info)
        self._home = [[400, 0, 300], [0, 0, -1, 0]]
        self._robot.set_cartesian(self._home)
        logger.debug('Leaving ABBRobotClientTestCase.setUp')
    
    def tearDown(self):
        """
        Clean up after each test
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.tearDown')
        self._robot.set_cartesian(self._home)
        del self._robot
        time.sleep(1)
        logger.debug('Leaving ABBRobotClientTestCase.tearDown')
    
    def test_async_set_cartesian(self):
        """
        Test asynchronous move to specified end effector pose
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.test_async_set_cartesian')
        target = [(np.array(self._home[0]) + [0, 0, 50]).tolist(), self._home[1]]
        t0 = time.clock()
        self._robot.async_set_cartesian(target)
        t1 = time.clock()
        self._robot.async_result()
        t2 = time.clock()
        actual =self._robot.get_cartesian()
        logger.debug('Actual pose = %s', str(actual))
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.5)
        self.assertPoseAlmostEqual(target, actual, 1, 0.01)
        logger.debug('Leaving ABBRobotClientTestCase.test_async_set_cartesian')

    def test_async_set_joints(self):
        """
        Test asynchronous setting of joint angles
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.test_async_set_joints')
        curr =self._robot.get_joints()
        target = (np.array(curr) + [5, 5, 5, 5, 5, 5]).tolist()
        t0 = time.clock()
        self._robot.async_set_joints(target)
        t1 = time.clock()
        self._robot.async_result()
        t2 = time.clock()
        actual =self._robot.get_joints()
        logger.debug('Actual pose = %s', str(actual))
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.5)
        self.assertJointsAlmostEqual(target, actual, 1)   
        logger.debug('Leaving ABBRobotClientTestCase.test_async_set_joints')
    
    def test_async_buffer_execute(self):
        """
        Test asynchronous moves to poses stored in buffer
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.test_async_buffer_execute')
        self._robot.clear_buffer()
        via1 = [(np.array(self._home[0]) + [0, 0, 50]).tolist(), self._home[1]]
        via2 = [(np.array(via1[0]) + [0, 50, 0]).tolist(), via1[1]]
        target = [(np.array(via2[0]) + [50, 0, 0]).tolist(), via2[1]]
        self._robot.buffer_set([via1, via2, target])
        t0 = time.clock()
        self._robot.async_buffer_execute()
        t1 = time.clock()
        self._robot.async_result()
        t2 = time.clock()
        actual =self._robot.get_cartesian()
        logger.debug('Actual pose = %s', str(actual))
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.5)
        self.assertPoseAlmostEqual(target, actual, 1, 0.01)  
        logger.debug('Leaving ABBRobotClientTestCase.test_async_buffer_execute')

    def test_async_move_circular(self):
        """
        Test asynchronous circular move to specified end effector pose
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering ABBRobotClientTestCase.test_async_move_circular')
        via = [(np.array(self._home[0]) + [0, 0, 50]).tolist(), self._home[1]]
        target = [(np.array(via[0]) + [0, 50, 0]).tolist(), via[1]]
        t0 = time.clock()
        self._robot.async_move_circular(via, target)
        t1 = time.clock()
        self._robot.async_result()
        t2 = time.clock()
        actual =self._robot.get_cartesian()
        logger.debug('Actual pose = %s', str(actual))
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.1)
        self.assertPoseAlmostEqual(target, actual, 1, 0.01)
        logger.debug('Leaving ABBRobotClientTestCase.test_async_move_circular')
        
#    def test_matlab_problem(self):
#        logger = logging.getLogger(__name__)
#        logger.debug('Entering ABBRobotClientTestCase.test_matlab_problem')        
#        self._robot.set_cartesian(self._home)
#        pose = [[360, -10, 91], [0, 0, -1, 0]]
#        self._robot.set_cartesian(pose)
#        time.sleep(1)
#        self._robot.clear_buffer()        
#        via = [(np.array(pose[0]) + [0, 0, -5]).tolist(), pose[1]]
#        target = [(np.array(via[0]) + [0, 0, 5]).tolist(), via[1]]
#        self._robot.buffer_set([via, target])
#        t0 = time.clock()
#        self._robot.async_buffer_execute()
#        t1 = time.clock()
#        self._robot.async_result()
#        t2 = time.clock()       
#        self.assertLess(t1 - t0, 0.01)
#        self.assertGreater(t2 - t1, 0.5)        
#        logger.debug('Leaving ABBRobotClientTestCase.test_matlab_problem')
        
if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr)
    logging.getLogger('tactile.robot.abb').setLevel(logging.DEBUG)
    logging.getLogger(__name__).setLevel(logging.DEBUG)
    
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ABBRobotClientTestCase))
    runner = unittest.TextTestRunner(stream=sys.stderr, descriptions=True, verbosity=1)
    runner.run(suite)    
#    unittest.main()

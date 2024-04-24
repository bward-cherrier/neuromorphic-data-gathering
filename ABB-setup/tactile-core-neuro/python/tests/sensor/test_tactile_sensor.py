# -*- coding: utf-8 -*-

import unittest
import sys
import logging
import time

from tactile.sensor.tactile_sensor_client import TactileSensorClient

class TactileSensorClientTestCase(unittest.TestCase):
    """
    Tests for TactileSensorClient in tactile_sensor.py
    """

    def setUp(self):
        """
        Initialise prior to each test
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering TactileSensorClientTestCase.setUp')
        self._sensor = TactileSensorClient(mask_type='circle',
                                           mask_centre=(310, 230),
                                           mask_radius=220)
        self._sensor.init_pins()
        logger.debug('Leaving TactileSensorClientTestCase.setUp')
        
    def tearDown(self):
        """
        Clean up after each test
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering TactileSensorClientTestCase.tearDown')        
        del self._sensor
        logger.debug('Leaving TactileSensorClientTestCase.tearDown')
        
    def test_tracking(self):
        """
        Test sensor pin tracking
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering TactileSensorClientTestCase.test_tracking')
        t0 = time.clock()
        self._sensor.async_track_pins()
        t1 = time.clock()
        time.sleep(1)
        t2 = time.clock()
        self._sensor.async_cancel()
        self._sensor.async_result()
        t3 = time.clock()
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.99)        
        self.assertLess(t3 - t2, 0.1)
        logger.debug('Leaving TactileSensorClientTestCase.test_tracking')
        
    def test_recording(self):
        """
        Test sensor pin recording
        """
        logger = logging.getLogger(__name__)
        logger.debug('Entering TactileSensorClientTestCase.test_recording')
        t0 = time.clock()
        self._sensor.async_record_pins()
        t1 = time.clock()
        time.sleep(1)
        t2 = time.clock()
        self._sensor.async_cancel()
        pins = self._sensor.async_result()
        t3 = time.clock()
        self.assertLess(t1 - t0, 0.01)
        self.assertGreater(t2 - t1, 0.99)
        self.assertLess(t3 - t2, 0.1)
        self.assertGreater(pins.shape[0], 27)
        self.assertLess(pins.shape[0], 33)
        logger.debug('Leaving TactileSensorClientTestCase.test_recording')    

if __name__ == '__main__':  
    logging.basicConfig(stream=sys.stderr)
    logging.getLogger('tactile.sensor.tactile_sensor').setLevel(logging.DEBUG)
    logging.getLogger(__name__).setLevel(logging.DEBUG)
    
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TactileSensorClientTestCase))
    runner = unittest.TextTestRunner(stream=sys.stderr, descriptions=True, verbosity=1)
    runner.run(suite)
#    unittest.main()

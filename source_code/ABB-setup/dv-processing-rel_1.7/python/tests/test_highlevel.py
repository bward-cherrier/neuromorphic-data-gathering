import unittest
import dv_processing as dv


class HighLevelTest(unittest.TestCase):

    def test_version(self):
        self.assertTrue(isinstance(dv.__version__, str))


if __name__ == '__main__':
    unittest.main()

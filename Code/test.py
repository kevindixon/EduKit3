import unittest
from mock import patch, call


@patch("RPi.GPIO")
class TestRobot(unittest.TestCase):
    import robot

    def test_init(self, mock_output):
        bot = robot.Robot()


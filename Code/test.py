import unittest
import time
from mock import patch, call, MagicMock

# Mock out the RPi module for test purposes
RPi_mock = MagicMock()
RPi_mock.GPIO = MagicMock()
modules = { 'RPi': RPi_mock, 'RPi.GPIO': RPi_mock.GPIO }
module_patcher = patch.dict('sys.modules', modules)
module_patcher.start()
# Can now import robot which depends on RPi.GPIO
import robot

class TestRobot(unittest.TestCase):

    def test_addCommands(self):
        bot = robot.Robot()
        bot.addStopMotorsCommand()
        self.assertEqual(bot.commandQueueLength(), 1)
        bot.addForwardsCommand()
        self.assertEqual(bot.commandQueueLength(), 2)

    def test_runCommands(self):
        bot = robot.Robot()
        bot.addStopMotorsCommand()
        bot.addForwardsCommand()
        bot.addForwardsCommand()
        bot.startQueue()
        self.assertTrue(bot.isQueueActive())


def main():
    unittest.main()

if __name__ == "__main__":
    main()

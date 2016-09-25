import unittest
import time
from datetime import datetime, timedelta
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

    def test_runCommandQueue(self):
        bot = robot.Robot()
        bot.addStopMotorsCommand()
        bot.addForwardsCommand()
        bot.addLeftCommand()
        bot.addRightCommand()
        bot.addBackwardsCommand()
#        print("\n".join("{}, {}, {}".format(f.__name__, a, k) for f, a, k in bot.getQueue()))
        bot.startQueue()
        self.assertTrue(bot.isQueueActive())

    def test_commandWithDelay(self):
        bot = robot.Robot()
        bot.addForwardsCommand(seconds = 2)
        start = datetime.now()
        bot.startQueue()
        while bot.isQueueActive():
            time.sleep(0.01)
        delta = datetime.now() - start
        self.assertTrue(delta > timedelta(seconds = 2))

def main():
    unittest.main()

if __name__ == "__main__":
    main()

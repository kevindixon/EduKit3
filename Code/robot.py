import RPi.GPIO as GPIO
import time
from datetime import datetime
import threading
from collections import deque

class Robot:

    # Motor control

    # Set variables for the GPIO motor pins
    PIN_MOTOR_A_FORWARDS = 9
    PIN_MOTOR_A_BACKWARDS = 10
    PIN_MOTOR_B_FORWARDS = 8
    PIN_MOTOR_B_BACKWARDS = 7

    # Setting the duty cycle to 0 means the motors will not turn
    STOP = 0

    # Line following

    # Set variables for the GPIO pins
    PIN_LINE_FOLLOWER = 25

    # Ultrasound
    # Define GPIO pins to use on the Pi
    PIN_ULTRASOUND_TRIGGER = 17
    PIN_ULTRASOUND_ECHO = 18

    OBSTACLE_DISTANCE_THREADHOLD_CM = 35.0
    OBSTACLE_AVOID_REVERSE_TIME_S = 0.5
    OBSTACLE_AVOID_TURN_TIME_S = 0.6
    OBSTACLE_AVOID_TURN_TIME_VARIANCE_S = 0.2

    def __init__(self):
        # Set up command queue
        self.commandQueue = deque()
        self.commandThread = threading.Thread(target = self._serviceQueue)
        self.stopQueue = True
        self.stopPatrol = True

        # Set the GPIO modes
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motors
        # Set the GPIO Pin mode
        GPIO.setup(Robot.PIN_MOTOR_A_FORWARDS, GPIO.OUT)
        GPIO.setup(Robot.PIN_MOTOR_A_BACKWARDS, GPIO.OUT)
        GPIO.setup(Robot.PIN_MOTOR_B_FORWARDS, GPIO.OUT)
        GPIO.setup(Robot.PIN_MOTOR_B_BACKWARDS, GPIO.OUT)

        # How many times to turn the pin on and off each second
        self.setPwmFrequency(20)
        self.setDutyCycleA(30)
        self.setDutyCycleB(29)

        # Start with a duty cycle of 0 (i.e. not moving)
        self.pwmMotorAForwards.start(Robot.STOP)
        self.pwmMotorABackwards.start(Robot.STOP)
        self.pwmMotorBForwards.start(Robot.STOP)
        self.pwmMotorBBackwards.start(Robot.STOP)

        # Line following
        # Set pin 25 as an input so its value can be read
        GPIO.setup(Robot.PIN_LINE_FOLLOWER, GPIO.IN)

        # Ultrasound
        # Set pins as output and input
        GPIO.setup(Robot.PIN_ULTRASOUND_TRIGGER, GPIO.OUT) # Trigger
        GPIO.setup(Robot.PIN_ULTRASOUND_ECHO, GPIO.IN) # Echo

    def __del__(self):
        self.stopMotors()
        GPIO.cleanup()

    def commandQueueLength(self):
        return len(self.commandQueue)

    def _addToCommandQueue(self, func, args=None, kwargs=None):
        self.commandQueue.appendleft((func, args, kwargs))

    def _serviceQueue(self):
        while True:
            if self.stopQueue:
                return
            try:
                func, args, kwargs = self.commandQueue.pop()
                func(*(args or []), **(kwargs or {}))
            except IndexError:
                break;

    def startQueue(self):
        if self.isQueueActive():
            return;
        self.stopQueue = False
        self.commandThread.start()

    def isQueueActive(self):
        return self.commandThread.isAlive()

    def getQueue(self):
        commands = []
        for cmd, args, kwargs in self.commandQueue:
            commands.append((cmd, args, kwargs))
        return commands

    def setDutyCycleA(self, dutyCycle):
        # How long the pin stays on each cycle, as a percent
        self.dutyCycleA = dutyCycle

    def setDutyCycleB(self, dutyCycle):
        # How long the pin stays on each cycle, as a percent
        self.dutyCycleB = dutyCycle

    def setPwmFrequency(self, frequency):
        # Set the GPIO to software PWM at 'frequency' Hertz
        self.pwmMotorAForwards = GPIO.PWM(Robot.PIN_MOTOR_A_FORWARDS, frequency)
        self.pwmMotorABackwards = GPIO.PWM(Robot.PIN_MOTOR_A_BACKWARDS, frequency)
        self.pwmMotorBForwards = GPIO.PWM(Robot.PIN_MOTOR_B_FORWARDS, frequency)
        self.pwmMotorBBackwards = GPIO.PWM(Robot.PIN_MOTOR_B_BACKWARDS, frequency)
        self.pwmMotorAForwards.start(Robot.STOP)
        self.pwmMotorABackwards.start(Robot.STOP)
        self.pwmMotorBForwards.start(Robot.STOP)
        self.pwmMotorBBackwards.start(Robot.STOP)

    def addStopMotorsCommand(self):
        self._addToCommandQueue(self.stopMotors)

    def stopMotors(self):
        # Turn all motors off
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)

    def addForwardsCommand(self, seconds = None):
        self._addToCommandQueue(self.forwards, kwargs={'seconds': seconds})

    def forwards(self, seconds = None):
        # Turn both motors forwards
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)
        if not seconds:
            return
        time.sleep(seconds)
        self.stopMotors()

    def addBackwardsCommand(self):
        self._addToCommandQueue(self.backwards)

    def backwards(self, seconds = None):
        # Turn both motors backwards
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)
        if not seconds:
            return
        time.sleep(seconds)
        self.stopMotors()

    def addRightCommand(self, seconds = None):
        self._addToCommandQueue(self.right, kwargs={'seconds': seconds})

    def right(self, seconds = None):
        # Turn left
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)
        if not seconds:
            return
        time.sleep(seconds)
        self.stopMotors()

    def addLeftCommand(self, seconds = None):
        self._addToCommandQueue(self.left, kwargs={'seconds': seconds})

    def left(self, seconds = None):
        # Turn Right
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)
        if not seconds:
            return
        time.sleep(seconds)
        self.stopMotors()

    #
    # Line follower
    #

    def getLineFollowSensorValue(self):
        return GPIO.input(Robot.PIN_LINE_FOLLOWER)

    def isLineFollowSensorOnBlack(self):
        return self.getLineFollowSensorValue() == 0

    #
    # Ultrasonic
    #

    def getObstacleDistance(self):
      # Send 10us pulse to trigger
      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, True)
      time.sleep(0.00001)
      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
      # Start the timer
      StartTime = time.time()
      StopTime = StartTime
      # The start time is reset until the Echo pin is taken high (==1)
      while GPIO.input(Robot.PIN_ULTRASOUND_ECHO)==0:
          StartTime = time.time()
          StopTime = StartTime
      # Stop when the Echo pin is no longer high - the end time
      while GPIO.input(Robot.PIN_ULTRASOUND_ECHO)==1:
        StopTime = time.time()
        # If the sensor is too close to an object, the Pi cannot
        # see the echo quickly enough, so it has to detect that
        # problem and say what has happened
        if StopTime - StartTime >= 0.04:
            return 0
      # Calculate pulse length
      elapsedTime = StopTime - StartTime
      # Distance pulse travelled in that time is
      # time multiplied by the speed of sound (cm/s)
      distance = elapsedTime * 34326
      # That was the distance there and back so halve the value
      distance = distance / 2
      return distance

    def _getRandomTurnTime(self):
        return random.uniform(
                       Robot.OBSTACLE_AVOID_TURN_TIME_S - Robot.BSTACLE_AVOID_TURN_TIME_VARIANCE_S,
                       Robot.OBSTACLE_AVOID_TURN_TIME_S + Robot.BSTACLE_AVOID_TURN_TIME_VARIANCE_S)

    def _avoidObstacle(self):
        # Get a varied turn time
        turnTime = self._getRandomTurnTime()
        while (leftDistance < Robot.OBSTACLE_DISTANCE_THREADHOLD_CM) and (rightDistance < Robot.OBSTACLE_DISTANCE_THREADHOLD_CM):
            # Back off a little
            self.backwards(Robot.OBSTACLE_AVOID_REVERSE_TIME_S)
            # Turn right
            self.right(turnTime)
            rightDistance = self.getObstacleDistance()
            # Turn back to the left
            self.left(turnTime * 2)
            leftDistance = self.getObstacleDistance()

        if leftDistance > rightDistance:
            return
        self.right(turnTime * 2)

    def addPatrolCommand(self):
        self.stopPatrol = False
        self._addToCommandQueue(self._patrol)

    def completePatrol(self):
        self.stopPatrol = True

    def _patrol(self):
        # Set trigger to False (Low)
        GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
        # Allow module to settle
        time.sleep(0.1)
        while True:
            if self.stopPatrol:
                break
            self.forwards()
            time.sleep(0.25)
            if self.getObstacleDistance() < Robot.OBSTACLE_DISTANCE_THREADHOLD_CM:
                self.stopMotors()
                self._avoidObstacle()

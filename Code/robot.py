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

    OBSTACLE_DISTANCE_THREADHOLD_CM = 15.0
    OBSTACLE_AVOID_REVERSE_TIME_S = 0.5
    OBSTACLE_AVOID_TURN_TIME_S = 0.25

    def __init__(self):
        # Set up command queue
        self.commandQueue = deque()
        self.commandThread = threading.Thread(target = self._serviceQueue)
        self.stopQueue = False
        self.isPatrolling = False

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
        self.setDutyCycleB(27)

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
        self.stopQueue = False
        self.isPatrolling = False
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

    def addForwardsCommand(self, seconds = 1):
        self._addToCommandQueue(self.forwards, kwargs={'seconds': seconds})

    def forwards(self, seconds = 1):
        # Turn both motors forwards
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)
        time.sleep(seconds)

    def addBackwardsCommand(self):
        self._addToCommandQueue(self.backwards)

    def backwards(self, seconds = 1):
        # Turn both motors backwards
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)
        time.sleep(seconds)

    def addLeftCommand(self, seconds = 1):
        self._addToCommandQueue(self.left, kwargs={'seconds': seconds})

    def left(self, seconds = 1):
        # Turn left
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)
        time.sleep(seconds)

    def addRightCommand(self, seconds = 1):
        self._addToCommandQueue(self.right, kwargs={'seconds': seconds})

    def right(self, seconds = 1):
        # Turn Right
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)
        time.sleep(seconds)

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
      # Set trigger to False (Low)
#      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
      # Allow module to settle
#      time.sleep(0.1)
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

    def avoidObstacle(self):
        # Back off a little
        self.backwards()
        time.sleep(Robot.OBSTACLE_AVOID_REVERSE_TIME_S)
        self.stopMotors()
        # Turn right
        self.right()
        time.sleep(Robot.OBSTACLE_AVOID_TURN_TIME_S)
        self.stopMotors()

    def patrol(self):
        self._addToCommandQueue(self._patrol)

    def _patrol(self):
        self.isPatrolling = True
        # Stop the command queue if it is active
#        if self.isQueueActive:
#            self.stopQueue = True
        # Patrol
        try:
            # Set trigger to False (Low)
            GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
            # Allow module to settle
            time.sleep(0.1)
            while True:
                self.forwards()
                if not self.isPatrolling:
                    break
                time.sleep(0.05)
                if self.getObstacleDistance() < Robot.OBSTACLE_DISTANCE_THREADHOLD_CM:
                    self.stopMotors()
                    self.avoidObstacle()
        except KeyboardInterrupt:
            pass
#        GPIO.cleanup()

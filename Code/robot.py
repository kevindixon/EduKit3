import RPi.GPIO as GPIO
import time

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

    def __init__(self):
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

    def stopMotors(self):
        # Turn all motors off
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)

    def forwards(self):
        # Turn both motors forwards
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)

    def backwards(self):
        # Turn both motors backwards
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)

    def left(self):
        # Turn left
        self.pwmMotorAForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorABackwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.dutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(Robot.STOP)

    def right(self):
        # Turn Right
        self.pwmMotorAForwards.ChangeDutyCycle(self.dutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBForwards.ChangeDutyCycle(Robot.STOP)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.dutyCycleB)

    #
    # Line follower
    #

    def getLineFollowSensorValue(self):
        return GPIO.input(Robot.PIN_LINE_FOLLOWER)

    def lineFollowerOnBlack(self):
        return self.getLineFollowSensorValue() == 0

    #
    # Ultrasonic
    #

    def getObstacleDistance(self):
      # Set trigger to False (Low)
      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
      # Allow module to settle
      time.sleep(0.5)
      # Send 10us pulse to trigger
      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, True)
      time.sleep(0.00001)
      GPIO.output(Robot.PIN_ULTRASOUND_TRIGGER, False)
      # Start the timer
      StartTime = time.time()
      # The start time is reset until the Echo pin is taken high (==1)
      while GPIO.input(Robot.PIN_ULTRASOUND_ECHO)==0:
          StartTime = time.time()
      # Stop when the Echo pin is no longer high - the end time
      while GPIO.input(Robot.PIN_ULTRASOUND_ECHO)==1:
          StopTime = time.time()
          # If the sensor is too close to an object, the Pi cannot
          # see the echo quickly enough, so it has to detect that
          # problem and say what has happened
          if StopTime-StartTime >= 0.04:
            return 0
      # Calculate pulse length
      elapsedTime = StopTime - StartTime
      # Distance pulse travelled in that time is
      # time multiplied by the speed of sound (cm/s)
      distance = elapsedTime * 34326
      # That was the distance there and back so halve the value
      distance = distance / 2
      return distance


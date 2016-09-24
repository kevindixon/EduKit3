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
        self.pwmMotorAForwards.start(Robot.STOP)
        self.pwmMotorABackwards.start(Robot.STOP)
        self.pwmMotorBForwards.start(Robot.STOP)
        self.pwmMotorBBackwards.start(Robot.STOP)
        # Set the GPIO to software PWM at 'frequency' Hertz
        self.pwmMotorAForwards = GPIO.PWM(Robot.PIN_MOTOR_A_FORWARDS, frequency)
        self.pwmMotorABackwards = GPIO.PWM(Robot.PIN_MOTOR_A_BACKWARDS, frequency)
        self.pwmMotorBForwards = GPIO.PWM(Robot.PIN_MOTOR_B_FORWARDS, frequency)
        self.pwmMotorBBackwards = GPIO.PWM(Robot.PIN_MOTOR_B_BACKWARDS, frequency)

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
# Line follower control
#

# Set variables for the GPIO pins
pinLineFollower = 25
'''
# Set pin 25 as an input so its value can be read
GPIO.setup(pinLineFollower, GPIO.IN)
try:
  # Repeat the next indented block forever
  while True:
    # If the sensor is Low (=0), it is above the black line
    if GPIO.input(pinLineFollower)==0:
      print('The sensor is seeing a black surface')
    # If not (else), print the following
    else:
      print('The sensor is seeing a white surface')
    # Wait, then do the same again
    time.sleep(0.2)
# If you press CTRL+C, cleanup and stop
except KeyboardInterrupt:
 GPIO.cleanup()
'''

#
# Ultrasonic
#

'''# Define GPIO pins to use on the Pi
pinTrigger = 17
pinEcho = 18
print("Ultrasonic Measurement")
# Set pins as output and input
GPIO.setup(pinTrigger, GPIO.OUT) # Trigger
GPIO.setup(pinEcho, GPIO.IN) # Echo

try:
  # Repeat the next indented block forever
  while True:
      # Set trigger to False (Low)
      GPIO.output(pinTrigger, False)
      # Allow module to settle
      time.sleep(0.5)
      # Send 10us pulse to trigger
      GPIO.output(pinTrigger, True)
      time.sleep(0.00001)
      GPIO.output(pinTrigger, False)
      # Start the timer
      StartTime = time.time()
      # The start time is reset until the Echo pin is taken high (==1)
      while GPIO.input(pinEcho)==0:
          StartTime = time.time()
      # Stop when the Echo pin is no longer high - the end time
      while GPIO.input(pinEcho)==1:
          StopTime = time.time()
          # If the sensor is too close to an object, the Pi cannot
          # see the echo quickly enough, so it has to detect that
          # problem and say what has happened
          if StopTime-StartTime >= 0.04:
              print("Hold on there! You're too close for me to see.")
              StopTime = StartTime
              break
      # Calculate pulse length
      ElapsedTime = StopTime - StartTime
      # Distance pulse travelled in that time is
      # time multiplied by the speed of sound (cm/s)
      Distance = ElapsedTime * 34326
      # That was the distance there and back so halve the value
      Distance = Distance / 2
      print("Distance: %.1f cm" % Distance)
      time.sleep(0.5)
# If you press CTRL+C, cleanup and stop
except KeyboardInterrupt:
  # Reset GPIO settings
  GPIO.cleanup()
'''

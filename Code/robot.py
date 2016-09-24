import RPi.GPIO as GPIO
import time

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#
# Motor control
#

# Set variables for the GPIO motor pins
pinMotorAForwards = 10
pinMotorABackwards = 9
pinMotorBForwards = 7
pinMotorBBackwards = 8

# How many times to turn the pin on and off each second
Frequency = 20
# How long the pin stays on each cycle, as a percent
DutyCycleA = 30
DutyCycleB = 30
# Setting the duty cycle to 0 means the motors will not turn
Stop = 0

# Set the GPIO Pin mode
GPIO.setup(pinMotorAForwards, GPIO.OUT)
GPIO.setup(pinMotorABackwards, GPIO.OUT)
GPIO.setup(pinMotorBForwards, GPIO.OUT)
GPIO.setup(pinMotorBBackwards, GPIO.OUT)

# Set the GPIO to software PWM at 'Frequency' Hertz
pwmMotorAForwards = GPIO.PWM(pinMotorAForwards, Frequency)
pwmMotorABackwards = GPIO.PWM(pinMotorABackwards, Frequency)
pwmMotorBForwards = GPIO.PWM(pinMotorBForwards, Frequency)
pwmMotorBBackwards = GPIO.PWM(pinMotorBBackwards, Frequency)

# Start the software PWM with a duty cycle of 0 (i.e. not moving)
pwmMotorAForwards.start(Stop)
pwmMotorABackwards.start(Stop)
pwmMotorBForwards.start(Stop)
pwmMotorBBackwards.start(Stop)

# Turn all motors off
def StopMotors():
  pwmMotorAForwards.ChangeDutyCycle(Stop)
  pwmMotorABackwards.ChangeDutyCycle(Stop)
  pwmMotorBForwards.ChangeDutyCycle(Stop)
  pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors forwards
def Forwards():
  pwmMotorAForwards.ChangeDutyCycle(DutyCycleA)
  pwmMotorABackwards.ChangeDutyCycle(Stop)
  pwmMotorBForwards.ChangeDutyCycle(DutyCycleB)
  pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors backwards
def Backwards():
  pwmMotorAForwards.ChangeDutyCycle(Stop)
  pwmMotorABackwards.ChangeDutyCycle(DutyCycleA)
  pwmMotorBForwards.ChangeDutyCycle(Stop)
  pwmMotorBBackwards.ChangeDutyCycle(DutyCycleB)

# Turn left
def Left():
  pwmMotorAForwards.ChangeDutyCycle(Stop)
  pwmMotorABackwards.ChangeDutyCycle(DutyCycleA)
  pwmMotorBForwards.ChangeDutyCycle(DutyCycleB)
  pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn Right
def Right():
  pwmMotorAForwards.ChangeDutyCycle(DutyCycleA)
  pwmMotorABackwards.ChangeDutyCycle(Stop)
  pwmMotorBForwards.ChangeDutyCycle(Stop)
  pwmMotorBBackwards.ChangeDutyCycle(DutyCycleB)

Forwards()
time.sleep(1) # Pause for 1 second

Forwards()
time.sleep(1) # Pause for 1 second

StopMotors()
GPIO.cleanup()

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

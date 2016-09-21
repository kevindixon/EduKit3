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

# Set the GPIO Pin mode
GPIO.setup(pinMotorAForwards, GPIO.OUT)
GPIO.setup(pinMotorABackwards, GPIO.OUT)
GPIO.setup(pinMotorBForwards, GPIO.OUT)
GPIO.setup(pinMotorBBackwards, GPIO.OUT)

def forward():
     GPIO.output(pinMotorAForwards, 1)
     GPIO.output(pinMotorABackwards, 0)
     GPIO.output(pinMotorBForwards, 1)
     GPIO.output(pinMotorBBackwards, 0)

def back():
     GPIO.output(pinMotorAForwards, 0)
     GPIO.output(pinMotorABackwards, 1)
     GPIO.output(pinMotorBForwards, 0)
     GPIO.output(pinMotorBBackwards, 1)

def stop():
     GPIO.output(pinMotorAForwards, 0)
     GPIO.output(pinMotorABackwards, 0)
     GPIO.output(pinMotorBForwards, 0)
     GPIO.output(pinMotorBBackwards, 0)

def spinleft():
     GPIO.output(9,1)
     GPIO.output(10,0)
     GPIO.output(7,1)
     GPIO.output(8,0)

def left():
     GPIO.output(9,1)
     GPIO.output(10,0)
     GPIO.output(7,0)
     GPIO.output(8,0)

def spinright():
     GPIO.output(9,0)
     GPIO.output(10,1)
     GPIO.output(7,0)
     GPIO.output(8,1)

stop()

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

# Define GPIO pins to use on the Pi
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

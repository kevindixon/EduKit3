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

stop()

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

#
# Line follower control
#

# Set variables for the GPIO pins
pinLineFollower = 25

# Set pin 25 as an input so its value can be read
GPIO.setup(pinLineFollower, GPIO.IN)
try:
  # Repeat the next indented block forever
  while True:
  # If the sensor is Low (=0), it’s above the black line
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

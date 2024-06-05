#!/usr/bin/env python

# Import required modules
import time
import RPi.GPIO as GPIO

# Declare the GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Drive the motor clockwise
GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(16, GPIO.OUT)

GPIO.output(20, GPIO.HIGH) # Set AIN1
# Set the motor speed
GPIO.output(16, GPIO.HIGH) # Set PWMA

# Disable STBY (standby)


# Wait 5 seconds
time.sleep(5)
GPIO.output(16, GPIO.LOW) # Set PWMA
GPIO.cleanup()

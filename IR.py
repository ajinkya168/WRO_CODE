import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the IR sensor pin
IR_PIN = 21

# Set up the IR sensor pin as an input
GPIO.setup(IR_PIN, GPIO.IN)

while True:
    # Read the IR sensor value
    ir_value = GPIO.input(IR_PIN)

    # White color typically reflects more IR light, so the sensor value will be low
    if ir_value == 0:
        print("White color detected")
    # Orange color typically reflects less IR light, so the sensor value will be high
    elif ir_value == 1:
        print("Orange color detected")
    else:
        print("Unknown color detected")

    # Wait for 0.5 seconds before taking the next reading
    time.sleep(0.5)

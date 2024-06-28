
import RPi.GPIO as GPIO
import time

# Set up GPIO pins for encoder and motor
ENC_A = 17
ENC_B = 23
MOTOR_DIR = 20
MOTOR_PWM = 12

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up encoder pins as inputs
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up motor pins as outputs
GPIO.setup(MOTOR_DIR, GPIO.OUT)
GPIO.setup(MOTOR_PWM, GPIO.OUT)

# Initialize motor direction and PWM
GPIO.output(MOTOR_DIR, GPIO.LOW)
pwm = GPIO.PWM(MOTOR_PWM, 50)  # 50 Hz PWM frequency
pwm.start(0)  # Start PWM with 0% duty cycle

# Initialize encoder variables
encoder_pos = 0
encoder_prev_A = GPIO.input(ENC_A)
encoder_prev_B = GPIO.input(ENC_B)

try:
    while True:
        # Read encoder pins
        encoder_A = GPIO.input(ENC_A)
        encoder_B = GPIO.input(ENC_B)

        # Calculate encoder position
        if encoder_A!= encoder_prev_A:
            if encoder_B!= encoder_prev_B:
                encoder_pos += 1
            else:
                encoder_pos -= 1

        # Update previous encoder values
        encoder_prev_A = encoder_A
        encoder_prev_B = encoder_B

        # Control motor based on encoder position
        if encoder_pos > 100:
            GPIO.output(MOTOR_DIR, GPIO.HIGH)
            pwm.ChangeDutyCycle(50)  # 50% duty cycle
        elif encoder_pos < -100:
            GPIO.output(MOTOR_DIR, GPIO.LOW)
            pwm.ChangeDutyCycle(50)  # 50% duty cycle
        else:
            pwm.ChangeDutyCycle(0)  # 0% duty cycle

        # Print encoder position
        print("Encoder position:", encoder_pos)

        # Wait for 10ms before reading encoder again
        time.sleep(0.01)

except KeyboardInterrupt:
    # Clean up GPIO pins
    GPIO.cleanup()






"""import gpiozero
from signal import pause
import time
# Define the encoder pins
encoder_a = gpiozero.DigitalInputDevice(17)  # Pin 17 for encoder A
#encoder_b = gpiozero.DigitalInputDevice(18)  # Pin 23 for encoder B
	
# Initialize the counter
counter = 0

def encoder_a_rise():
    global counter

    if encoder_b.value:
        counter += 1
    else:
        counter -= 1


def encoder_counter():
	global counter
	if encoder_a.value:
		counter += 1
	else:
		counter -= 1

'''def encoder_b_rise():
    global counter

    if encoder_a.value:
        counter -= 1
        
    else:
        counter += 1'''

# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_counter
#encoder_b.when_activated = encoder_b_rise

print("Encoder counter started. Press Ctrl+C to exit.")

try:
	while True:
		time.sleep(0.1)
		print("counter A: {}".format(counter))
except KeyboardInterrupt:
    print("Encoder counter stopped.")
    print("Final counter value:", counter)
    
    """



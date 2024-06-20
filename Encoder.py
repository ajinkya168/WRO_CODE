import gpiozero
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
encoder_a.when_activated = ecndoer_count
encoder_b.when_activated = encoder_b_rise

print("Encoder counter started. Press Ctrl+C to exit.")

try:
	while True:
		time.sleep(0.1)
		print("counter A: {}".format(counter))
except KeyboardInterrupt:
    print("Encoder counter stopped.")
    print("Final counter value:", counter)
    
    



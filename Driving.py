import RPi.GPIO as GPIO
import pigpio
import time
import os

servo = 23

previous_state = 0
button_state = 0
button  = False

GPIO.setmode(GPIO.BCM)

pwm = pigpio.pi()

pwm.set_mode(servo, pigpio.OUTPUT)

pwm.set_PWM_frequency(servo, 50)

GPIO.setup(5 , GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23

GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(16, GPIO.OUT)



	
def setAngle(angle):
	pwm.set_servo_pulsewidth(servo, 500 + round(angle*11.11)) # 0 degree





while True:

	previous_state = button_state
	button_state = GPIO.input(5)
	setAngle(90)
	print(button)
	if(previous_state == 1 and button_state == 0):
		button = not(button)
		
	if(button):
			GPIO.output(16, GPIO.HIGH) # Set PWMA
			GPIO.output(20, GPIO.HIGH) # Set AIN1
			time.sleep(2)
			setAngle(45)
			time.sleep(2)
			setAngle(90)
	else:
		GPIO.output(16, GPIO.LOW) # Set PWMA'

		
GPIO.cleanup()	


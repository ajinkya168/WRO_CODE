import RPi.GPIO as GPIO
import pigpio

import os

servo = 23

GPIO.setmode(GPIO.BCM)
pwm = pigpio.pi()

pwm.set_mode(servo, pigpio.OUTPUT)

pwm.set_PWM_frequency(servo, 50)


GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(16, GPIO.OUT)


def correctAngle():
	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correcion = 0
	totalError = 0
	prevError = 0
	kp = 0.5
	ki = 0
	kd = 0
	error_gyro = currentAngle - 0
	setPoint_flag =  0

	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp * error_gyro
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm;
	if (setPoint_flag == 0) :
		if (correction > 10) :
			correction = 10
		elif (correction < -10): 
			correction = -10

	else: 
		if (correction > 35) :
			correction = 35
		elif (correction < -35) :
			correction = -35
			
	

	prevErrorGyro = error_gyro

	SetAngle(90 + correction)
	
	
def SetAngle(angle):
	pwm.set_servo_pulsewidth(servo, 500 + round(angle*11.11)) # 0 degree


	

import time
import board
import busio
import RPi.GPIO as GPIO
import pigpio
import time

from math import atan2, sqrt, pi

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)


servo = 23




pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)

pwm.set_PWM_frequency( servo, 50 )


def correctAngle(currentAngle):
	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correcion = 0
	totalError = 0
	prevError = 0
	kp = 1
	ki = 0.5
	kd = 0
	setPoint_flag =  0

	error_gyro = currentAngle - 0

	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp * error_gyro
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm;
	if (setPoint_flag == 0) :
		if (correction > 25) :
			correction = 25
		elif (correction < -25): 
			correction = -25

	else: 
		if (correction > 45) :
			correction = 45
		elif (correction < -45) :
			correction = -45
			
	

	prevErrorGyro = error_gyro

	setAngle(91 + correction)



def setAngle(angle):
	pwm.set_servo_pulsewidth( servo, 500 + round(angle*11.11) )
	
def find_heading(dqw, dqx, dqy, dqz):
    norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = atan2(t3, t4)
    
   
    yaw = yaw_raw * 180.0 / pi
    yaw = yaw - 180

    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise
  


while True:
	time.sleep(0.5)
	print("Gyro:")
	#gyro_x, gyro_y, gyro_z = bno.gyro # pylint:disable=no-member
	#print("X: %0.6f Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
	#print("")
	quat_i, quat_j, quat_k, quat_real = bno.quaternion
	print("Rotation Vector Quaternion:")
	#quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
	#print(
	#"I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
	#)
	#print("")
	heading = find_heading(quat_real, quat_i, quat_j, quat_k)
	print("Heading using rotation vector:", heading)
	correctAngle(heading)
	

GPIO.cleanup()




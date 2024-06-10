import time
import board
import busio
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
	






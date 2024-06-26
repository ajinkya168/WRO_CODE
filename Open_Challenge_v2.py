import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time
import multiprocessing
import pigpio
import board
import busio
from math import atan2, sqrt, pi
import os
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

time.sleep(5)
glob = 0

GPIO.setmode(GPIO.BCM)

GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(12, GPIO.OUT)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23
pwm12 = GPIO.PWM(12, 100)

#pwm = pigpio.pi()


#Parameters for servo
servo = 23

RX_Head = 16
RX_Left = 9
RX_Right = 25
#pi = pigpio.pi()

'''pwm.set_mode(servo, pigpio.OUTPUT)

pwm.set_PWM_frequency(servo, 50)

pwm.set_mode(RX_Head, pigpio.INPUT)
pwm.set_mode(RX_Left, pigpio.INPUT)
pwm.set_mode(RX_Right, pigpio.INPUT)
pwm.bb_serial_read_open(RX_Head, 115200)
pwm.bb_serial_read_open(RX_Left, 115200)
pwm.bb_serial_read_open(RX_Right, 115200)'''
#Define object specific variables for green  
dist = 15
focal = 1120
pixels = 30
width = 4

dist1 = 0
dist2 = 0

currentAngle =0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0
kp = 0.6
ki = 0.1
kd = 0.5
setPoint_flag =  0


#pi = pigpio.pi()

distance_head = 0
distance_left = 0
distance_right = 0


def getTFminiData():

  #while True:

	
	#while True:
		time.sleep(0.01)	#change the value if needed
		#(count, recv) = pi.bb_serial_read(RX)
		(count_head, recv_head) = pwm.bb_serial_read(RX_Head)
		(count_left, recv_left) = pwm.bb_serial_read(RX_Left)
		(count_right, recv_right) = pwm.bb_serial_read(RX_Right)
		if count_head > 8:
		  for i in range(0, count_head-9):
		    if recv_head[i] == 89 and recv_head[i+1] == 89: # 0x59 is 89
		      checksum = 0
		      for j in range(0, 8):
		        checksum = checksum + recv_head[i+j]
		      checksum = checksum % 256
		      if checksum == recv_head[i+8]:
		        global distance_head
		        distance_head = recv_head[i+2] + recv_head[i+3] * 256
		        strength_head = recv_head[i+4] + recv_head[i+5] * 256
		        #print("distance_head : ", distance_head)
		        
		        
		if count_left > 8:
		  for i in range(0, count_left-9):
		    if recv_left[i] == 89 and recv_left[i+1] == 89: # 0x59 is 89
		      checksum = 0
		      for j in range(0, 8):
		        checksum = checksum + recv_left[i+j]
		      checksum = checksum % 256
		      if checksum == recv_left[i+8]:
		        global distance_left
		        distance_left = recv_left[i+2] + recv_left[i+3] * 256
		        strength_left = recv_left[i+4] + recv_left[i+5] * 256
		        #print("distance_left : ", distance_left) 
		        
		if count_right > 8:
		  for i in range(0, count_right-9):
		    if recv_right[i] == 89 and recv_right[i+1] == 89: # 0x59 is 89
		      checksum = 0
		      for j in range(0, 8):
		        checksum = checksum + recv_right[i+j]
		      checksum = checksum % 256
		      if checksum == recv_right[i+8]:
		        global distance_right
		        distance_right = recv_right[i+2] + recv_right[i+3] * 256
		        strength_right = recv_right[i+4] + recv_right[i+5] * 256
		        #print("distance_right : ", distance_right)




	
def correctAngle(setPoint_gyro, left, right):
	global glob
	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0
	
	quat_i, quat_j, quat_k, quat_real = bno.quaternion
	heading = find_heading(quat_real, quat_i, quat_j, quat_k)
	glob = heading

	'''if not right and not left or right and not left:
		if((heading > 180 and setPoint_gyro < 180)):
			heading =  heading - 360

	elif left and not right:
		if((heading < 180 and abs(setPoint_gyro) < 0)):
			heading = heading - 360'''
			
			


	#if(heading > 18)
	#print("Heading :", heading) 
	error_gyro = heading - setPoint_gyro

	if((error_gyro > 180 )):
		error_gyro =  error_gyro - 360



	#print("Error : ", error_gyro)
	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp * error_gyro
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm;
	#print("correction 1: ", correction)
				
	'''if(heading > 180 and setPoint_gyro < 180):	
		heading =  heading - 360'''	
	if (setPoint_flag == 0) :
		if (correction > 30) :
			correction = 30
		elif (correction < -30): 
			correction = -30

	else:
	 
		if (correction > 35) :
			correction = 35
		elif (correction < -35) :
			correction = -35

	#print("correction: ", e)	
	#print("heading: {}, error: {}, correction: {}, left:{}, right: {}".format(heading, error_gyro, correction, left, right))
	prevErrorGyro = error_gyro
	setAngle(90 - correction)

	

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
    
   
    yaw = yaw_raw * 180 / pi
    yaw = yaw - 180

    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise




def setAngle(angle):
	pwm.set_servo_pulsewidth(servo, 500 + round(angle*11.11)) # 0 degree



def BrightnessContrast(brightness=0): 
  
    # getTrackbarPos returns the  
    # current position of the specified trackbar. 
    brightness = cv2.getTrackbarPos('Brightness', 
                                    'GEEK') 
      
    contrast = cv2.getTrackbarPos('Contrast', 
                                  'GEEK') 
      
    effect = controller(img, 
                        brightness, 
                        contrast) 
  
    # The function imshow displays  
    # an image in the specified window 
    cv2.imshow('Effect', effect) 





#find the distance from then camera
def get_dist(rectange_params,image, block):
	#find no of pixels covered
	pixels = rectange_params[1][0]
	global dist1
	global dist2
	dist1 = (width*focal)/pixels
	dist2 = (width*focal)/pixels
	#print(pixels)



    #calculate distance
	if block == "green":

		image = cv2.putText(image, 'Distance from Camera GREEN in CM :', org, font,  
		1, color, 2, cv2.LINE_AA)

		image = cv2.putText(image, str(dist1), (110,50), font,  
		fontScale, color, 1, cv2.LINE_AA)

	else:

		image = cv2.putText(image, 'Distance from Camera RED in CM :', (700, 20), font,  
		1, color, 2, cv2.LINE_AA)

		image = cv2.putText(image, str(dist2), (1000,50), font,  
		fontScale, color, 1, cv2.LINE_AA)
		#Wrtie n the image
	return image


#Extract Frames 
#cap = cv2.VideoCapture(0)

#basic constants for opencv Functs
kernel = np.ones((3,3),'uint8')
font = cv2.FONT_HERSHEY_SIMPLEX 
org = (0,20)  
fontScale = 0.6 
color = (0, 0, 255) 
thickness = 2

def resize_final_img(x,y,*argv):
    images  = cv2.resize(argv[0], (x, y))
    for i in argv[1:]:
        resize = cv2.resize(i, (x, y))
        images = np.concatenate((images,resize),axis = 1)
    return images

#loop to capture video frames
def Live_Feed(distance, block):
	print("Image Process started")
	picam2 = Picamera2()
	picam2.preview_configuration.main.size = (1280,720)
	picam2.preview_configuration.main.format = "RGB888"
	picam2.preview_configuration.align()
	picam2.configure("preview")
	picam2.start()

	green_present = False
	red_present = False
	
	cv2.namedWindow('Object Dist Measure ',cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Object Dist Measure ', 900,800)

	while True:



		img= picam2.capture_array()

		hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		hsv_img1 = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

		#predefined mask for green colour detection





		lower = np.array([49, 118, 69])
		upper = np.array([71, 172, 119])
		mask = cv2.inRange(hsv_img, lower, upper)
		mask = cv2.dilate(mask, kernel, iterations=3)
		mask = cv2.erode(mask, kernel, iterations=3)
	

		lower1 = np.array([167, 172, 110])
		upper1 = np.array([178, 205, 133])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)
		mask1 = cv2.dilate(mask1, kernel, iterations=3)
		mask1 = cv2.erode(mask1, kernel, iterations=3)
	
		#Remove Extra garbage from image
		d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,iterations = 5)
		d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel,iterations = 5)
		#final_img = resize_final_img(300,300, mask, d_img)
		#final_img1 = resize_final_img(300,300, mask1, d_img1)

		#cv2.imshow("Image", final_img )
		#cv2.imshow("Image1", final_img1 )
		#find the histogram
		cont,hei = cv2.findContours(d_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cont = sorted(cont, key = cv2.contourArea, reverse = True)[:1]
	
		cont1,hei1 = cv2.findContours(d_img1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cont1 = sorted(cont1, key = cv2.contourArea, reverse = True)[:1]

		if(len(cont) == 0):
			print("Cant find contour for green....")
			green_present = False
		else:
	
			max_cnt = max(cont, key = cv2.contourArea)
			green_present = True
			
		if(len(cont1) == 0):
			print("Cant find contour for red....")
			red_present = False
		else:
	
			max_cnt1 = max(cont1, key = cv2.contourArea)
			red_present = True
			
		#max_cnt1 = max(cont1, key = cv2.contourArea)
		if(red_present and green_present):
			if(cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1)):
				if (cv2.contourArea(max_cnt)>100 and cv2.contourArea(max_cnt)<306000):

					#Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect,img, "green")
					distance.value = dist1
					block.value = 1		    

		#check for contour area
			else:
				if (cv2.contourArea(max_cnt1)>100 and cv2.contourArea(max_cnt1)<306000):

					#Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect1,img, "red")            
					distance.value = dist2
					block.value = 2
		elif(red_present):
				if (cv2.contourArea(max_cnt1)>100 and cv2.contourArea(max_cnt1)<306000):

					#Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect1,img, "red")            
					distance.value = dist2
					block.value = 2			
		elif(green_present):
				if (cv2.contourArea(max_cnt)>100 and cv2.contourArea(max_cnt)<306000):

					#Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect,img, "green")
					distance.value = dist1
					block.value = 1			

						   
		cv2.imshow('Object Dist Measure ',img)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print(dist2)


	cv2.destroyAllWindows()
	picam2.stop()
	GPIO.cleanup()


def redDrive():
	correctAngle(30)
	time.sleep(0.5)
	correctAngle(-30)
	time.sleep(0.8)
	correctAngle(0)

	print("Red Detect")
	
	
def greenDrive():

	print("Green Detect")
	correctAngle(-30)
	time.sleep(0.5)
	correctAngle(30)
	time.sleep(0.8)
	correctAngle(0)	



	
def servoDrive(distance, block, pwm):
	print("ServoProcess started")
	global heading
	global distance_right
	global distance_head
	global distance_left
	pwm.set_mode(servo, pigpio.OUTPUT)

	pwm.set_PWM_frequency(servo, 50)

	pwm.set_mode(RX_Head, pigpio.INPUT)
	pwm.set_mode(RX_Left, pigpio.INPUT)
	pwm.set_mode(RX_Right, pigpio.INPUT)
	pwm.bb_serial_read_open(RX_Head, 115200)
	pwm.bb_serial_read_open(RX_Left, 115200)
	pwm.bb_serial_read_open(RX_Right, 115200)
	
	
	previous_state = 0
	button_state = 0
	button  = False
	start_time = 0
	pwm12.start(0)
	power = 0
	turn_flag = False
	heading_angle = 0
	target_angle = 0
	trigger = False
	counter = 0
	left_flag = False
	right_flag = False
	correctAngle(0, left_flag, right_flag)
	while True:
		init_flag = False
		
		correctAngle(heading_angle, left_flag, right_flag)
		time.sleep(0.1)
		getTFminiData()

		previous_state = button_state
		button_state = GPIO.input(5)
		#print("Trigger: ",trigger)
		
		if(previous_state == 1 and button_state == 0):
			button = not(button)
			init_flag = True


			print("Button is pressed")
		if(button):
			#bno.init()
			if not right_flag and not left_flag:
				if(distance_right > 100):
					right_flag = True
					
									#pwm.bb_serial_read_close(RX_Left)
				elif(distance_left > 100):
					left_flag = True
					#pwm.bb_serial_read_close(RX_Right)

			print(trigger)
			
			correctAngle(heading_angle, left_flag, right_flag)

			if(init_flag):
				for power in np.arange(0,100, 0.01):
					pwm12.ChangeDutyCycle(power)# Set PWMA
					#pass
				init_flag = False
			pwm12.ChangeDutyCycle(power)

			GPIO.output(20, GPIO.HIGH) # Set AIN1
			if(right_flag):
				distance_left = -1
				if(counter == -1):
					if(time.time() - start_time > 1.1):
						power = 0
				if(counter == 12 ):
				
					if(distance_head < 175 and distance_right < 85 and not trigger):
						start_time = time.time()
						counter = -1

				if(distance_right > 100 and not trigger and distance_head < 75):
						#time.sleep(0.5)
						
						if((glob >= 0 and glob <=15) or (glob >= 343 and glob <= 370)):
							heading_angle = 90
							counter = counter + 1


						elif(glob >= 78 and glob <= 105):
							heading_angle = 180
							counter = counter + 1

							
						elif(glob >= 165 and glob <= 190):
							heading_angle = 270
							counter = counter + 1

							
						elif(glob >= 255 and glob <= 283):
							heading_angle = 0
							counter = counter + 1

						trigger = True
				if(distance_right < 85 and distance_head > 75):
					trigger = False
			elif(left_flag):
				distance_right = -1
				if(counter == -1):

					if(time.time() - start_time > 1.1):
						power = 0
				if(counter == 12 ):
				
					if(distance_head < 175 and distance_left < 85 and not trigger):
						start_time = time.time()
						counter = -1		
				if(distance_left > 150 and not trigger and distance_head < 75):
						#time.sleep(0.5)
						
						if((glob >= 0 and glob <=15) or (glob >= 343 and glob <= 370)):
							heading_angle = -90
							counter = counter + 1 

						
						elif(glob >= 255 and glob <= 283): #78 and glob <= 105
							heading_angle = -180
							counter = counter + 1

							
						elif(glob >= 165 and glob <= 190): #165 and glob <= 190
							heading_angle = -270
							counter = counter + 1

							
						elif(glob >= 78 and glob <= 105): #255 and glob <= 283
							heading_angle = -0
							counter = counter + 1

						trigger = True
		

				elif(distance_left < 100 and distance_head > 75):
					trigger = False
		
				
			print("Trigger : {}, counter : {}, distance_head : {}, distance_left: {}, distance_right: {}, Theta: {}, IMU : {}".format(trigger, counter, distance_head, distance_left, distance_right, heading_angle, glob)) 
		else:
			if(init_flag):
				init_flag = False
			pwm12.ChangeDutyCycle(0)
			heading_angle = 0
			counter = 0
			left_flag = False
			right_flag = False
			correctAngle(heading_angle, left_flag, right_flag)
			
			
			
			
											





if __name__ == '__main__':

	try:
		pwm = pigpio.pi()
		distance = multiprocessing.Value('f', 0.0)
		block = multiprocessing.Value('i', 0)

		S = multiprocessing.Process(target = servoDrive, args = (distance, block, pwm ))


		S.start()

	except KeyboardInterrupt:
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		pwm12.ChangeDutyCycle(0)
		GPIO.cleanup()			


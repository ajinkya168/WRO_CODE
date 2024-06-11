import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time
import board
import busio
from math import atan2, sqrt, pi
import serial
import threading


time.sleep(5)




GPIO.setmode(GPIO.BCM)


GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(16, GPIO.OUT)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23


flag = False


picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280,720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

#Define object specific variables for green  
dist = 15
focal = 1120
pixels = 30
width = 4

dist1 = 0
dist2 = 0

MAX_BUFF_LEN = 255
SETUP 		 = False
port 		 = None

transfer = False
while(not SETUP):
	try:
		# 					 Serial port(windows-->COM), baud rate, timeout msg
		port = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

	except: # Bad way of writing excepts (always know your errors)
		if(time.time() - prev > 2): # Don't spam with msg
			print("No serial detected, please plug your uController")
			prev = time.time()

	if(port is not None): # We're connected
		SETUP = True


def read_ser(num_char = 1):
	string = port.read(num_char)
	return string.decode()

# Write whole strings
def write_ser(cmd):
	cmd = str(cmd)
	cmd = cmd + '\n'
	port.write(cmd.encode())

# Super loop

	string = read_ser(MAX_BUFF_LEN)
	if(len(string)):
		print(string)

	cmd = 2 # Blocking, there're solutions for this ;)
	if(cmd):
		write_ser(cmd)


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


cv2.namedWindow('Object Dist Measure ',cv2.WINDOW_NORMAL)
cv2.resizeWindow('Object Dist Measure ', 700,600)



def Image_Reading():
	previous_state = 0
	button_state = 0
	button  = False
	while True:

		previous_state = button_state
		button_state = GPIO.input(5)

			
		if(previous_state == 1 and button_state == 0):
			button = not(button)
			print("Button is pressed")

		#loop to capture video frames

		img= picam2.capture_array()

		hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		hsv_img1 = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)


		#predefined mask for green colour detection
		lower = np.array([55, 37, 0])
		upper = np.array([87, 162, 255])
		mask = cv2.inRange(hsv_img, lower, upper)

		lower1 = np.array([128, 121, 62])
		upper1 = np.array([179, 255, 255])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)



		#Remove Extra garbage from image
		d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,iterations = 5)
		d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel,iterations = 5)


		#find the histogram
		cont,hei = cv2.findContours(d_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cont = sorted(cont, key = cv2.contourArea, reverse = True)[:1]

		cont1,hei1 = cv2.findContours(d_img1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cont1 = sorted(cont1, key = cv2.contourArea, reverse = True)[:1]



		for cnt, cnt1 in zip(cont, cont1):
		#check for contour area
			if(cv2.contourArea(cnt) > cv2.contourArea(cnt1)):
				if (cv2.contourArea(cnt)>100 and cv2.contourArea(cnt)<306000):

					#Draw a rectange on the contour
					rect = cv2.minAreaRect(cnt)
					box = cv2.boxPoints(rect) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect,img, "green")
		    

		#check for contour area
			else:
				if (cv2.contourArea(cnt1)>100 and cv2.contourArea(cnt1)<306000):

					#Draw a rectange on the contour
					rect1 = cv2.minAreaRect(cnt1)
					box = cv2.boxPoints(rect1) 
					box = np.int0(box)
					cv2.drawContours(img,[box], -1,(255,0,0),3)

					img = get_dist(rect1,img, "red")            
		   




		if cv2.waitKey(1) & 0xFF == ord('q'):
			break_flag = True;
			break
			
		cv2.imshow('Object Dist Measure ',img)
		print(dist2)
		
		if(button):

			#GPIO.output(16, GPIO.HIGH) # Set PWMA
			#GPIO.output(20, GPIO.HIGH) # Set AIN1	
			if(dist1 < 20 or dist2 < 20) :

				print("inside")



		
				

		else:

			GPIO.output(16, GPIO.LOW) # Set PWMA

def Serial_Comm():
	while(1):
		cmd = dist2 # Blocking, there're solutions for this ;)
		string = read_ser(MAX_BUFF_LEN)
		if(len(string)):
			print(string)

		if(cmd):
			write_ser(cmd)
		


t1 = threading.Thread(target = Image_Reading)
t2 = threading.Thread(target = Serial_Comm)

t1.start()
t2.start()


cv2.destroyAllWindows()
picam2.stop()
GPIO.cleanup()
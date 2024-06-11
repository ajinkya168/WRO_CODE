from gpiozero import Servo
from time import sleep

servo = Servo(23)

try:
	while True:
		with open("Data.txt","r") as f:
			if f.read() == 1:
				servo.max()
			else:
				servo.min()
		#servo.min()
		#sleep(0.5)
		#servo.mid()
		#sleep(0.5)
		#servo.max()
		#sleep(0.5)
except KeyboardInterrupt:
	print("Program stopped")

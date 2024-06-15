import cv2
import numpy as np
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280,720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
# Create a VideoCapture object
#

# Define a function to get the HSV values at a specific point
def get_hsv_values(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_values = hsv[y, x]
        print(f'HSV values at ({x}, {y}): {hsv_values}')
        print(f'HSV max: {hsv_values}')
        print(f'HSV min: {hsv_values}')

# Create a window and set the callback function
cv2.namedWindow('Live Feed')
cv2.setMouseCallback('Live Feed', get_hsv_values)

while True:
    frame = picam2.capture_array()

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
    # Display the frame
    cv2.imshow('Live Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

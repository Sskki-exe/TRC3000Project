#Spin servo @Dylan 
#Take an image with PiCam @Calvin Medeira 
import picamera
import time
picture_number = 0
def camera_picture():
    camera = PiCamera()
    camera.start_preview() # Display image
    sleep(5)               # Give time for camera to sense light levels
    camera.capture('/home/pi/Desktop/image%s.jpg', picture_number) # Save picture on desktop
    picture_number += 1
    camera.stop_preview()  # Close image


#Read load sensor @pat 
compensation = 0
def readLS():
    value=0 #fix this
    return value
#Calibrate load sensor @Harry 
def calibrateLS():
    compensation = readLS()

#Read IMU @Kaelan

#Process image
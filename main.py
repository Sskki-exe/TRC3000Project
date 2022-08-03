#Spin servo @Dylan 
#Take an image with PiCam @Calvin Medeira 
#Read load sensor @pat 

# must do "pip install hx711-rpi-py==1.57.0" first 
# rpi gpio has to be also installed
from HX711 import *

hx = SimpleHX711(2, 3, -370, -367471)
hx.zero()
while True:
  print(hx.weight(35))


compensation = 0
def readLS():
    value=0 #fix this
    return value
#Calibrate load sensor @Harry 
def calibrateLS():
    compensation = readLS()

#Read IMU @Kaelan

#Process image
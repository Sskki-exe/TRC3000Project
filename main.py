#Spin servo @Dylan 
import RPi.GPIO as GPIO
from time import sleep

gpio_num=11 #Can change??

GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_num, GPIO.OUT)
pwm=GPIO.PWM(gpio_num, 50) #Second parameter = freq.
pwm.start(0)

def setAngle(angle):
    duty = angle / 18 + 3
    GPIO.output(gpio_num, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(gpio_num, False)
    pwm.ChangeDutyCycle(duty)

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

#Read IMU @Kaelan-------------------------------------------
# Help from https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module
# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
# https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

import smbus            #import SMBus module of I2C
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

    
def read_raw_data(addr):
    #The data straight from the MPU6050 is in 2 8bit words. Combine them to get the raw value
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def getValue(variable):
    #Outputs the value we should be using
    
    g = 9.80665 #Newton/kg
    accel_scale_factor = 16384 #corresponds to acceleration sensitivity of +- 2 gforce
    ang_vel_scale_factor = 131 #corresponds to gyroscope sensitivity of +-250 degrees/s
    #determine whether acceleration or tilting
    if (variable==ACCEL_XOUT_H || variable == ACCEL_YOUT_H || variable == ACCEL_ZOUT_H):
        return read_raw_data(variable)/accel_scale_factor
        #returns value in g force
    elif (variable==GYRO_XOUT_H || variable == GYRO_YOUT_H || variable == GYRO_ZOUT_H):
        return read_raw_data(variable) /ang_vel_scale_factor
        #returns value in degrees/second
    else:
        print("Variable not suitable for function")
        #if we reach a variable not meant for this function
        

#Process image

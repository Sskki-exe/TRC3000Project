#Leave comment when not in raspberry pi

#Import relevant files
from email.mime import image
import RPi.GPIO as GPIO
import smbus
from time import sleep
from hx711py.hx711 import HX711
import picamera
import cv2
import numpy as np
from sklearn.cluster import KMeans
import sys
import math
from datetime import datetime

#Spin servo @Dylan--------------------------------------------------------
servo_pin=17 #Signal pin for servo
currentAngle=0 #Initial Angle

GPIO.setmode(GPIO.BCM) #Refers to Physical Pin 11
GPIO.setup(servo_pin, GPIO.OUT)
pwm=GPIO.PWM(servo_pin, 50) #Second parameter = freq.
pwm.start(0)

def setAngle(angle):
    """angle: Desired angle of the servo [0, 180]\n
    setAngle sets the angle of the servo
    """
    global servo_pin, currentAngle
    duty = angle / 18 + 3
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.2)
    pwm.ChangeDutyCycle(0)
    GPIO.output(servo_pin, False)
    currentAngle=angle
setAngle(currentAngle) #Ensures starting at initial angle
def moveServo(angle, steps,delay):
    """angle: Desired angle of the servo [0, 180]\n
    steps: Number of steps towards final position\n
    delay: Delay between steps (seconds)\n
    moveServo allows the change of angle in the Servo over time with a finite number of steps
    """
    stepSize=(angle-currentAngle)/steps
    for i in range(1,steps+1):
        setAngle(currentAngle+stepSize*i)
        sleep(delay)

    currentAngle=angle


# sudo pigpiod
#from gpiozero.pins.pigpio import PiGPIOFactory
#servo=Servo(servo_pin, pin_factory=factory)
#servo.min()
#servo.max()
#servo.mid()


#Read load sensor @pat ------------------------------------------------------------------------------
# ref:https://tutorials-raspberrypi.com/digital-raspberry-pi-scale-weight-sensor-hx711/
# reference unit has been obtained using a known weight

referenceUnit = 1
hx = HX711(5, 6) # data is connected to pin 6 and sck to pin 5 and
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(referenceUnit)
hx.reset()

# this function is used to tare the load cell
def calibrateLS(knownWeight):
    """
    Title: Calibrate Load Cell
    Description: used initialise, tare and calibrate load cell
    """
    tareLS()
    #print(" enter known weight and place weight on the scale")
    #knownWeight=input()
    givenWeight= readLS()
    referenceUnit = givenWeight/float(knownWeight)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()


def tareLS():
    """
    Title: read load cell
    Description: thi function is used to determine weight on the load cell
    output: this function outputs the weight on the lod cell in grams
    """
    hx.tare() # tare the scale

    print("Tare done! ")



def readLS():
    val = hx.get_weight(5)
    hx.power_down()
    hx.power_up()
    sleep(0.1)
    return val

#Read IMU @Kaelan-----------------------------------------------------------------------------------------------------------
#IMU is called the MPU6050
# Help from https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module
# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
# https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

MPU_initialised = False #global boolean variable used to determine if MPU is already initialised, to stop from recalling it again


def MPU_create_registers():
    """
    Title: MPU6050 Inertia Measurement Unit Register Creation
    Description: This function creates a set of global variables that correspond to the registers for the MPU6050
    """
    global PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, INT_ENABLE, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H, IMU_Device_Address
    PWR_MGMT_1   = 0X6B
    SMPLRT_DIV   = 0X19
    CONFIG       = 0X1A
    GYRO_CONFIG  = 0X1B
    INT_ENABLE   = 0X38
    ACCEL_XOUT_H = 0X3B
    ACCEL_YOUT_H = 0X3D
    ACCEL_ZOUT_H = 0X3F
    GYRO_XOUT_H  = 0X43
    GYRO_YOUT_H  = 0X45
    GYRO_ZOUT_H  = 0X47
    IMU_Device_Address = 0x68   # MPU6050 device address

def MPU_init():
    """
    Title: MPU6050 Intertia Measurement Unit Initialiser
    Description: This function initialises the MPU6050 such that the relevant registers are written to and can be read
    """
    global bus, MPU_initialised
    
    if MPU_initialised==True:
        print("Already Initialised")
        return None
    
    print("Initialising MPU6050")
    #Create registers used for MPU6050
    MPU_create_registers()
    
    
    bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
    #If not working solve by https://github.com/johnbryanmoore/VL53L0X_rasp_python/issues/13
    #write to sample rate register
    bus.write_byte_data(IMU_Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(IMU_Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(IMU_Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(IMU_Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(IMU_Device_Address, INT_ENABLE, 1)
    
    #Calibrate Device
    MPU_calibrate()
    MPU_initialised = True
    print("MPU6050 initialisation complete")
    
def MPU_read_raw_data(addr):
    """
    Title: MPU6050 Inertia Measurement Unit Raw data Reader
    Description: This function reads the raw data given from the MPU6050. The MPU6050 outputs a 16 bit value given from two 8 bit words registers. This function reads those and combines them to output a signed unscaled 16 bit value.
    Inputs: addr - corresponds to the variable/register we are retrieving the value of. This is the acceleration of or rotation about x,y or z axis.
    Outputs: value - Outputs the signed unscaled value read from the MPU6050 for the given input
    """
    #The data straight from the MPU6050 is in 2 8bit words. Combine them to get the raw value
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(IMU_Device_Address, addr)
    low = bus.read_byte_data(IMU_Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def MPU_scaledValue(variable):
    """
    Title: MPU6050 Get Value
    Description: THis function obtains the scaled signed value for a given variable input
    Inputs: variable - corresponds to variable/register we are retriving the value of. This is the acceleration of, or rotation about the x,y or z axis.
    Outputs: Returns the scaled signed value for the given variable outputed by the MPU6050
    Note: This is for use between function, and does not produce a value to be used by users
    """
    
    accel_scale_factor = 16384 #corresponds to acceleration sensitivity of +- 2 gforce
    ang_vel_scale_factor = 131 #corresponds to gyroscope sensitivity of +-250 degrees/s
    
    #determine whether acceleration or tilting
    if (variable==ACCEL_XOUT_H or variable == ACCEL_YOUT_H or variable == ACCEL_ZOUT_H):
        return MPU_read_raw_data(variable)/accel_scale_factor
        #returns value in g force which is m/s^2
    elif (variable== GYRO_XOUT_H or variable == GYRO_YOUT_H or variable == GYRO_ZOUT_H):
        return MPU_read_raw_data(variable) /ang_vel_scale_factor
        #returns value in degrees/second
    else:
        print("Variable not suitable for function")
        #if we reach a variable not meant for this function
def MPU_getValue(variable):
    """
    Title: MPU6050 Inertia Measurement Unit Get Value
    Description: This function returns the the user friendly value of the variable inserted into it, rounded to 2 decimal places including the offsets and sensitivity factors. This value differs from MPU_scaledValue since it is a value that includes the offset and is rounded, being intended for presentation and not use from other functions
    Inputs: variable - takes a string input of the shortened value name from the list of aX, aY, aZ, wX, wY, wZ
    Outputs: returns the scaled, calibrated value for the given variable rounded to 2 decimal places
    """
    #Ensure we use the calibrated offsets
    global aX_offset, aY_offset, aZ_offset, wX_offset, wY_offset, wZ_off
    #Given the input variable,return the scaled value after adding the offset and rounding to 2 decimal places.
    if variable=='aX':
        return round(MPU_scaledValue(ACCEL_XOUT_H) + aX_offset,2)
    elif variable == 'aY':
        return round(MPU_scaledValue(ACCEL_YOUT_H) + aY_offset,2)
    elif variable == 'aZ':
        return round(MPU_scaledValue(ACCEL_ZOUT_H) + aZ_offset,2)
    elif variable == 'wX':
        return round(MPU_scaledValue(GYRO_XOUT_H) + wX_offset,2)
    elif variable == 'wY':
        return round(MPU_scaledValue(GYRO_YOUT_H) + wY_offset,2)
    elif variable == 'wZ':
        return round(MPU_scaledValue(GYRO_ZOUT_H) + wZ_offset,2)
    else:
        #error message if we do not enter the correct input
        print("Wrong input type. Did you put in string version?")
        
def MPU_tiltAngles():
    """
    Title: MPU6050 Inertia Measurement Unit Tilt Angle Finder
    Description: This function takes into account the gforce given from the MPU6050 from gravity, and convert it into the amount of tilt of the xy plane. We specifically do not include tilt of the xz or yz plane because does not interfere with our experiment.
    Outputs:
        tiltX - the amount of tilt that occurs around the y axis. E.g. a positive tiltX means we are tilting in the direction of the positive x axis.
        tiltY - the amount of tilt that occurs around the x axis. E.g. a positive tiltY means we are tilting in the direction of the positive y axis
    """
    #Obtain the raw acceleration values of xyz directions including offset
    x = MPU_scaledValue(ACCEL_XOUT_H) + aX_offset
    y = MPU_scaledValue(ACCEL_YOUT_H) + aY_offset
    z = MPU_scaledValue(ACCEL_ZOUT_H) + aZ_offset
    #If input is too insignificant, set it to 0. This helps with accuracy since small reductions in value change the angle greatly
    if abs(x)<0.01:
        x = 0
    if abs(y) <0.01:
        y=0
    if abs(z)<0.01:
        z=0
    #create the total vector via pythagoras
    total_vec = math.sqrt(x*x + y*y +z*z)
    #find roll and ptich and round to 2 decimal places
    roll = round(math.degrees(math.asin(x/total_vec)),2)
    pitch = round(math.degrees(math.asin(y/total_vec)),2)
    return roll,pitch
    
def MPU_calibrate():
    """
    Title: MPU6050 Inertia Measurement Unit Calibration
    Description: This function calibrates the MPU6050 to reduce errors in tilt
    Outputs: Note doesn't return a value but creates global variables
        aX_offset   - value to offset acceleration in x direction
        aY_offset   - value to offset acceleration in y direction
        aZ_offset   - value to offset acceleration in z direction
        wX__offset  - value to offset angular velocity around x axis
        wY_offset   - value to offset angular velocity around y axis
        wZ__offset  - value to offset angular velocity around z axis
    """
    #Warning before calibration
    print("Please ensure device is sitting on a flat stable surface")
    #Creating global values to offset in other functions
    global aX_offset, aY_offset, aZ_offset, wX_offset, wY_offset, wZ_offset
    #Indication of Calibration
    print("Calibrating MPU6050 Now")
    #We obtain the current values of acceleration of and angular velocity around xyz, then set the offset of the difference between what they are and there original set position
    aX = MPU_scaledValue(ACCEL_XOUT_H)
    aX_offset = 0-aX
    aY = MPU_scaledValue(ACCEL_YOUT_H)
    aY_offset = 0-aY
    aZ = MPU_scaledValue(ACCEL_ZOUT_H)
    aZ_offset = -1-aZ
    wX = MPU_scaledValue(GYRO_XOUT_H)
    wX_offset = 0-wX
    wY = MPU_scaledValue(GYRO_YOUT_H)
    wY_offset = 0-wY
    wZ = MPU_scaledValue(GYRO_ZOUT_H)
    wZ_offset = 0-wZ
    print("MPU6050 Calibration Complete")
    
def MPU_test(numTimes):
    """
    Title: MPU6050 Tester
    Description: This function is a testing function used to output the 6 possible values expected from the MPU6050 as well as the 2 tilt values. It can loops it a set number of times or infinitely, and prints the 8 values to the screen in order of
        AccelerationX  AccelerationY  AccelerationZ angularVelocityX   angularVelocityY   angularVelocityZ roll(tilt of X axis) pitch(tilt of Y axis)
        To print infinitely type 'inf' as the input
    Inputs: numTimes - the number of times we print values to the console
    """
    global MPU_initialised
    if MPU_initialised == False:
        MPU_init()
   
    print('{0:>8} {1:>8} {2:>8} {3:>8} {4:>8} {5:>8} {6:>8} {7:>8}'.format('aX','aY','aZ','wX','wY','wZ','tiltX','tiltY'))
    if (numTimes == 'inf'):
        while(True):
            print('{0:8} {1:8} {2:8} {3:8} {4:8} {5:8} {6:8} {7:8}'.format(MPU_getValue('aX'),MPU_getValue('aY'),MPU_getValue('aZ'),MPU_getValue('wX'),MPU_getValue('wY'),MPU_getValue('wZ'),MPU_tiltAngles()[0],MPU_tiltAngles()[1]))
    else:
        for x in range(numTimes):
            print('{0:8} {1:8} {2:8} {3:8} {4:8} {5:8} {6:8} {7:8}'.format(MPU_getValue('aX'),MPU_getValue('aY'),MPU_getValue('aZ'),MPU_getValue('wX'),MPU_getValue('wY'),MPU_getValue('wZ'),MPU_tiltAngles()[0],MPU_tiltAngles()[1]))

def MPU_getOutputs():
    """
    Title: MPU6050 Inertia Measurement Unit Get Outputs
    Description: This function is used to output the user-friendly values of acceleration in gforce for th x,y and z axes. And the roll(tilt around the x axis) and the pitch (tilt of the y axis) measured in degrees
    Outputs: returns an array of the values in order of
                AccelerationX AccelerationY AccelerationY Roll(TiltX) Pitch(TiltY)
    """
    return MPU_getValue('aX'),MPU_getValue('aY'),MPU_getValue('aZ'),MPU_tiltAngles()[0],MPU_tiltAngles()[1]
#--------------------------------------------------
#Acess USB Camera since PiCamera Port is broken on our RaspberryPi
cam = cv2.VideoCapture(0) #allows us to take photos using the USB Camera

def takePic():
    """
    Title: Take Picture
    Description: This function uses the usb camera we have connected to the raspberry pi and uses it to take photos. We are only using the usb webcam because the camera port on our raspberry pi is damaged and does not work
    Outputs: Does not return anything from function. However places a photo with current date and time into a folder called ProjectPics
    """
    global cam #ensure uses global variable
    now = datetime.now()
    print('Taking Photo')
    while True:
        ret,image = cam.read() #read the image and save it 'image'
        #cv2.imshow('ImageTest',image) #display photo
        #Code to ensure that the photos are correctly taken
        k = cv2.waitKey(1)
        if k!=1:
            break
    cv2.imwrite(('/home/raspberry/ProjectPics/'+now.strftime("%Y-%m-%d_%H:%M")+'.jpg'),image) #save photo to specified folder with current date and time
    cam.release() #stop using camera
    #cv2.destroyAllWindows() #destroy window displaying photo
    print('Photo taken and saved successfully')

#--------------------------------------------------
#Acceleration test
#NOT COMPLETE
def accelerationTest(lowerTol,sampleTol,initialSteps,initialDelay):
    """
    lowerTol: Amount of weight before "something" is detected
    sampleTol: Lower tolerance of the sample weight
    initialSteps: Amount of steps to intially use
    initialDelay: Length of delay between steps

    returns the maximum acceleration of the sample until the sample falls off
    """
    while readLS()<lowerTol:
        sleep(1)
    sampleWeight=readLS()-sampleTol
    while readLS()>sampleWeight:
        moveServo(180,initialSteps,initialDelay)
        initialDelay=initialDelay-1

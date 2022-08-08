#Spin servo @Dylan 
import RPi.GPIO as GPIO
import smbus
from time import sleep
from hx711 import HX711


servo_pin=11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm=GPIO.PWM(servo_pin, 50) #Second parameter = freq.
pwm.start(0)
def setAngle(angle):
    duty = angle / 18 + 3
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    pwm.ChangeDutyCycle(0)
    GPIO.output(servo_pin, False)


# sudo pigpiod
#from gpiozero.pins.pigpio import PiGPIOFactory
#servo=Servo(servo_pin, pin_factory=factory)
#servo.min()
#servo.max()
#servo.mid()



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



def readLS():
hx = HX711(5, 6)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(referenceUnit)

hx.reset()

hx.tare()

print("Tare done! Add weight now...")
val = hx.get_weight(5)
print(val)


#Read IMU @Kaelan-------------------------------------------
#IMU is called the MPU6050
# Help from https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module
# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
# https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf


def MPU_create_registers():
    #some MPU6050 Registers and their Address
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

def MPU_Init(): #initialises the MPU6050
    bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
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
    
    accel_scale_factor = 16384 #corresponds to acceleration sensitivity of +- 2 gforce
    ang_vel_scale_factor = 131 #corresponds to gyroscope sensitivity of +-250 degrees/s
    
    #determine whether acceleration or tilting
    if (variable==ACCEL_XOUT_H or variable == ACCEL_YOUT_H or variable == ACCEL_ZOUT_H):
        return read_raw_data(variable)/accel_scale_factor
        #returns value in g force which is m/s^2
    elif (variable==GYRO_XOUT_H or variable == GYRO_YOUT_H or variable == GYRO_ZOUT_H):
        return read_raw_data(variable) /ang_vel_scale_factor
        #returns value in degrees/second
    else:
        print("Variable not suitable for function")
        #if we reach a variable not meant for this function

#Process image

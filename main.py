#Spin servo @Dylan 
from email.mime import image
import RPi.GPIO as GPIO
import smbus
from time import sleep
from HX711 import *
import picamera
import cv2
import numpy as np
from sklearn.cluster import KMeans


servo_pin=11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm=GPIO.PWM(servo_pin, 50) #Second parameter = freq.
pwm.start(0)

currentAngle=0

def setAngle(angle):
    duty = angle / 18 + 3
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    pwm.ChangeDutyCycle(0)
    GPIO.output(servo_pin, False)
    currentAngle=angle

def moveServo(angle, steps,delay):
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



# Take an image with PiCam @Calvin Medeira 
# Ref: https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
def record_image(dir='/home/pi/Desktop/', image_name="image.jpg"):
    camera = PiCamera()
    #camera.start_preview() # Display image
    #sleep(5)               # Give time for camera to sense light levels
    camera.capture(dir + image_name) # Save picture on desktop
    #camera.stop_preview()  # Close image

## Image processing

# Determines average color of an image
# Refs: https://www.delftstack.com/howto/python/opencv-average-color-of-image/#:~:text=NumPy%20in%20Python.-,Use%20the%20average()%20Function%20of%20NumPy%20to%20Find%20the,the%20total%20number%20of%20elements.
def average_color(path="/home/pi/Desktop/image.jpg"):
    # Read image and store it in a matrix of RGB triplet values
    img = cv2.imread(path)
    # Find average of rgb values of matrix
    average_color_row = np.average(img, axis=0)
    average_color = np.average(average_color_row, axis=0)
    print(average_color)

    # Dispaly image of average color
    display_avg_color = np.ones((312, 312, 3), dtype=np.unit8)
    display_avg_color[:, :] = average_color

    cv2.imshow('Source image', img)
    cv2.imshow('Average Color', display_avg_color)
    #cv2.waitKey(0)


# Determines dominant color of an image
# ref: https://www.delftstack.com/howto/python/opencv-average-color-of-image/#:~:text=NumPy%20in%20Python.-,Use%20the%20average()%20Function%20of%20NumPy%20to%20Find%20the,the%20total%20number%20of%20elements.
def dominant_color(path="/home/pi/Desktop/image.jpg"):
    # Load image
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    reshape_img = image.reshape((image.shape[0] * image.shape[1], 3))

    # Display dominant colors Present in the image
    KM_cluster = KMeans(n_clusters=5).fit(reshape_img)
    C_labels = np.arange(0, len(np.unique(KM_cluster.labels_)) + 1)
    (C_hist, _) = np.histogram(KM_cluster.labels_, bins = C_labels)
    C_hist = C_hist.astype("float")
    C_hist /= C_hist.sum()

    rect_color = np.zeros((50, 300, 3), dtype=np.uint8)
    img_colors = sorted([(percent, color) for (percent, color) in zip(C_hist, KM_cluster.cluster_centers_)])
    start = 0
    for (percent, color) in img_colors:
        print(color, "{:0.2f}%".format(percent * 100))
        end = start + (percent * 300)
        cv2.rectangle(rect_color, (int(start), 0), (int(end), 50), \
                      color.astype("uint8").tolist(), -1)
        start = end
    
    rect_color = cv2.cvtColor(rect_color, cv2.COLOR_RGB2BGR)
    cv2.imshow('visualize_Color', rect_color)
    cv2.waitKey()


# Checks for color change
def color_change():
    pass

    

#Read load sensor @pat 

# must do "pip install hx711-rpi-py==1.57.0" first 
# rpi gpio has to be also installed

def readLS():
    hx = SimpleHX711(2, 3, -370, -367471)

    # set the scale to output weights in ounces
    hx.setUnit(Mass.Unit.OZ)

    # constantly output weights using the median of 35 samples

    print(hx.weight(35)) #eg. 1.08 oz

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
    global bus
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
    
def read_raw_data(addr):
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
        
def MPU_test(numTimes):
    for x in range(numTimes):
        print(x," ",getValue(ACCEL_XOUT_H)," ",getValue(ACCEL_YOUT_H)," ",getValue(ACCEL_ZOUT_H)," ",getValue(GYRO_XOUT_H)," ",getValue(GYRO_YOUT_H)," ",getValue(GYRO_ZOUT_H))

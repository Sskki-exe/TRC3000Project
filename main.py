#Spin servo @Dylan 
from email.mime import image
import RPi.GPIO as GPIO
import smbus
from time import sleep
from hx711 import HX711
import picamera
import cv2
import numpy as np
from sklearn.cluster import KMeans
import sys


servo_pin=11 #Signal pin for servo
currentAngle=0 #Initial Angle

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm=GPIO.PWM(servo_pin, 50) #Second parameter = freq.
pwm.start(0)

def setAngle(angle):
    """angle: Desired angle of the servo [0, 180]\n
    setAngle sets the angle of the servo
    """
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


# this function is used to clean up LS after use
def cleanAndExit():
    print("Cleaning...")


    GPIO.cleanup()
            
    print("Bye!")
    sys.exit()

# ref:https://tutorials-raspberrypi.com/digital-raspberry-pi-scale-weight-sensor-hx711/
def readLS():
    
    referenceUnit = 8882
    hx = HX711(5, 6) # data is connected to pin 6 and sck to pin 5
    hx.set_reading_format("MSB", "LSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()
    hx.tare() # tare the scale 

    print("Tare done! Add weight now...")
# print 10 diffrent weights
    for i in range(10):
        try:

            val = hx.get_weight(5)
            print(val)


            hx.power_down()
            hx.power_up()
            sleep(0.1)

        except (KeyboardInterrupt, SystemExit):
            cleanAndExit()


#Read IMU @Kaelan-------------------------------------------
#IMU is called the MPU6050
# Help from https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module
# https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
# https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf


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

def MPU_Init(): 
    """
    Title: MPU6050 Intertia Measurement Unit Initialiser
    Description: This function initialises the MPU6050 such that the relevant registers are written to and can be read
    """
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

def MPU_getValue(variable):
    """
    Title: MPU6050 Get Value
    Description: THis function obtains the scaled signed value for a given variable input
    Inputs: variable - corresponds to variable/register we are retriving the value of. This is the acceleration of, or rotation about the x,y or z axis.
    Outputs: Returns the scaled signed value for the given variable outputed by the MPU6050
    """
    
    accel_scale_factor = 16384 #corresponds to acceleration sensitivity of +- 2 gforce
    ang_vel_scale_factor = 131 #corresponds to gyroscope sensitivity of +-250 degrees/s
    
    #determine whether acceleration or tilting
    if (variable==ACCEL_XOUT_H or variable == ACCEL_YOUT_H or variable == ACCEL_ZOUT_H):
        return MPU_read_raw_data(variable)/accel_scale_factor
        #returns value in g force which is m/s^2
    elif (variable==GYRO_XOUT_H or variable == GYRO_YOUT_H or variable == GYRO_ZOUT_H):
        return MPU_read_raw_data(variable) /ang_vel_scale_factor
        #returns value in degrees/second
    else:
        print("Variable not suitable for function")
        #if we reach a variable not meant for this function
        
def MPU_test(numTimes):
    """
    Title: MPU6050 Tester
    Description: This function is a testing function used to output the 6 possible values expected from the MPU6050. It loops it a set number of times and prints the 6 values to the screen in order of
        AccelerationX  AccelerationY  AccelerationZ RotationX   RotationY   RotationZ
    Inputs: numTimes - the number of times we print values to the console
    """
    for x in range(numTimes):
        print(x," ",MPU_getValue(ACCEL_XOUT_H)," ",MPU_getValue(ACCEL_YOUT_H)," ",MPU_getValue(ACCEL_ZOUT_H)," ",MPU_getValue(GYRO_XOUT_H)," ",MPU_getValue(GYRO_YOUT_H)," ",MPU_getValue(GYRO_ZOUT_H))





#Acceleration test
#NOT COMPLETE
def accelerationTest(lowerTol,sampleTol,initialSteps,initialDelay)
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


    
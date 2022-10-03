from time import sleep
import cv2
import numpy as np
from sklearn.cluster import KMeans
import os
from datetime import datetime
import mediapipe as mp
from matplotlib import pyplot as plt
from PIL import Image

# Take an image with PiCam @Calvin Medeira 
# Ref: https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
def record_image(path="/home/raspberry/ProjectPics"):
    """
    Title: WebCam record image
    Description: This function will save a picture in 
    Inputs: addr - corresponds to the variable/register we are retrieving the value of. This is the acceleration of or rotation about x,y or z axis.
    Outputs: value - Outputs the signed unscaled value read from the MPU6050 for the given input
    """
    camera = cv2.VideoCapture(0)
    now = datetime.now()
    
    result, image = camera.read()
    cv2.waitKey(10)
    dir = os.path.join(path, now.strftime("%d%m%Y_%H-%M-%S")+".jpg")
    dir = dir.replace("\\", "/")
    print(dir)
    cv2.imwrite(dir, image)
    return image

## Image processing

# Determines average color of an image
# Refs: https://www.delftstack.com/howto/python/opencv-average-color-of-image/#:~:text=NumPy%20in%20Python.-,Use%20the%20average()%20Function%20of%20NumPy%20to%20Find%20the,the%20total%20number%20of%20elements.
def average_color():
    # Read image and store it in a matrix of RGB triplet values
    img = cv2.imread("image.jpg")
    # Find average of rgb values of matrix
    average_color_row = np.average(img, axis=0)
    average_color = np.average(average_color_row, axis=0)
    print(average_color)

    # Dispaly image of average color
    display_avg_color = np.ones((312, 312, 3), dtype=np.uint8)
    display_avg_color[:, :] = average_color

    cv2.imshow('Source image', img)
    cv2.imshow('Average Color', display_avg_color)
    cv2.waitKey(0)


# Determines dominant color of an image
# ref: https://www.delftstack.com/howto/python/opencv-average-color-of-image/#:~:text=NumPy%20in%20Python.-,Use%20the%20average()%20Function%20of%20NumPy%20to%20Find%20the,the%20total%20number%20of%20elements.
def dominant_color(path):
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
    # start = 0
    # for (percent, color) in img_colors:
    #     if (np.array_equal(np.floor(color), [0, 0, 0]) == False):
    #         print(color, "{:0.2f}%".format(percent * 100))
    #         end = start + (percent * 300)
    #         cv2.rectangle(rect_color, (int(start), 0), (int(end), 50), \
    #                      img_colors[-1][1].astype("uint8").tolist(), -1)
    #         start = end
    cv2.rectangle(rect_color, (int(start), 0), (int(end), 50), \
                    img_colors[-1][1].astype("uint8").tolist(), -1)
    rect_color = cv2.cvtColor(rect_color, cv2.COLOR_RGB2BGR)
    #cv2.imshow('Source image', cv2.imread("image.jpg"))
    cv2.imshow('visualize_Color', rect_color)
    cv2.waitKey(0)


def remove_background_fs(path):
    # path to input image specified and
    # image is loaded with imread command
    image = cv2.imread(path)

    # create a simple mask image similar
    # to the loaded image, with the
    # shape and return type
    mask = np.zeros(image.shape[:2], np.uint8)

    # specify the background and foreground model
    # using numpy the array is constructed of 1 row
    # and 65 columns, and all array elements are 0
    # Data type for the array is np.float64 (default)
    backgroundModel = np.zeros((1, 65), np.float64)
    foregroundModel = np.zeros((1, 65), np.float64)

    # define the Region of Interest (ROI)
    # as the coordinates of the rectangle
    # where the values are entered as
    # (startingPoint_x, startingPoint_y, width, height)
    # these coordinates are according to the input image
    # it may vary for different images
    rectangle = (265, 330, 216, 98)

    # apply the grabcut algorithm with appropriate
    # values as parameters, number of iterations = 3
    # cv2.GC_INIT_WITH_RECT is used because
    # of the rectangle mode is used
    cv2.grabCut(image, mask, rectangle,
                backgroundModel, foregroundModel,
                3, cv2.GC_INIT_WITH_RECT)

    # In the new mask image, pixels will
    # be marked with four flags
    # four flags denote the background / foreground
    # mask is changed, all the 0 and 2 pixels
    # are converted to the background
    # mask is changed, all the 1 and 3 pixels
    # are now the part of the foreground
    # the return type is also mentioned,
    # this gives us the final mask
    mask2 = np.where((mask == 2)|(mask == 0), 0, 1).astype('uint8')

    # The final mask is multiplied with
    # the input image to give the segmented image.
    src = image * mask2[:, :, np.newaxis]
    # Convert image to image gray
    tmp = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

    # Applying thresholding technique
    _, alpha = cv2.threshold(tmp, 0, 255, cv2.THRESH_BINARY)

    # Using cv2.split() to split channels 
    # of coloured image
    b, g, r = cv2.split(src)

    # Making list of Red, Green, Blue
    # Channels and alpha
    rgba = [b, g, r, alpha]

    # Using cv2.merge() to merge rgba
    # into a coloured/multi-channeled image
    dst = cv2.merge(rgba, 4)
    cv2.imwrite("Light_LS1HCover.png", dst)

dominant_color("test_cropped.png")
from email.mime import image
from time import sleep
import cv2
import numpy as np
from sklearn.cluster import KMeans

# Take an image with PiCam @Calvin Medeira 
# Ref: https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
def record_image(path="image.jpg"):
    """
    Title: WebCam record image
    Description: This function will save a picture in 
    Inputs: addr - corresponds to the variable/register we are retrieving the value of. This is the acceleration of or rotation about x,y or z axis.
    Outputs: value - Outputs the signed unscaled value read from the MPU6050 for the given input
    """
    camera = cv2.VideoCapture(0)
    result, image = camera.read()
    cv2.imwrite("image1.jpg", image)

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
def dominant_color():
    # Load image
    image = cv2.imread("image.jpg")
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
    cv2.imshow('Source image', cv2.imread("image.jpg"))
    cv2.imshow('visualize_Color', rect_color)
    cv2.waitKey(0)

average_color()
dominant_color()
# Checks for color change
def color_change():
    pass
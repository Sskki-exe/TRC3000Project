import cv2
import numpy as np
from sklearn.cluster import KMeans
import os
from datetime import datetime
from matplotlib import pyplot as plt
import matplotlib
from PIL import Image

# Take an image with PiCam @Calvin Medeira 
# Ref: https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
def record_image(path="/home/raspberry/ProjectPics"):
    """
    Title: WebCam record image
    Description: This function will save a picture in user defined path
    Inputs:
    Outputs: path of saved image
    """
    camera = cv2.VideoCapture(0)
    now = datetime.now()
    
    _, image = camera.read()
    cv2.waitKey(10)
    dir = os.path.join(path, now.strftime("%d%m%Y_%H-%M-%S")+".jpg")
    dir = dir.replace("\\", "/")
    print(dir)
    cv2.imwrite(dir, image)
    return dir

def remove_background_fs(path):
    """
    Title: Remove Background (Foreground segmentation)
    Description: This function will remove the background of an image (ie isolate the sample).
    Inputs: Image
    Outputs: path of the inputed image with its background removed.
    """
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
    cv2.imwrite("removed_back.png", dst)

# Determines dominant color of an image
# ref: https://www.delftstack.com/howto/python/opencv-average-color-of-image/#:~:text=NumPy%20in%20Python.-,Use%20the%20average()%20Function%20of%20NumPy%20to%20Find%20the,the%20total%20number%20of%20elements.
def dominant_color(path):
    """
    Title: Dominant Color
    Description: This function will determine the dominant color of an image
    Inputs: Image
    Outputs: Dominant color as RGB list
    """
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    reshape_img = image.reshape((image.shape[0] * image.shape[1], 3))

    # Display dominant colors Present in the image
    KM_cluster = KMeans(n_clusters=5).fit(reshape_img)
    C_labels = np.arange(0, len(np.unique(KM_cluster.labels_)) + 1)
    (C_hist, _) = np.histogram(KM_cluster.labels_, bins = C_labels)
    C_hist = C_hist.astype("float")
    C_hist /= C_hist.sum()

    dominant_color = sorted(list(zip(C_hist, KM_cluster.cluster_centers_)))[-2][1]
    # rect_color = np.zeros((50, 300, 3), dtype=np.uint8)
    # img_colors = sorted([(percent, color) for (percent, color) in zip(C_hist, KM_cluster.cluster_centers_)])
    # start = 0
    # for (percent, color) in img_colors:
    #     if (np.array_equal(np.floor(color), [0, 0, 0]) == False):
    #         print(color, "{:0.2f}%".format(percent * 100))
    #         end = start + (percent * 300)
    #         cv2.rectangle(rect_color, (int(start), 0), (int(end), 50), \
    #                      img_colors[-1][1].astype("uint8").tolist(), -1)
    #         start = end
    # cv2.rectangle(rect_color, (int(start), 0), (int(end), 50), \
    #                 img_colors[-1][1].astype("uint8").tolist(), -1)
    # rect_color = cv2.cvtColor(rect_color, cv2.COLOR_RGB2BGR)
    # #cv2.imshow('Source image', cv2.imread("image.jpg"))
    # cv2.imshow('visualize_Color', rect_color)
    # cv2.waitKey(0) 
    return np.ceil(dominant_color)

def sample_color():
    """
    Title:Image Processing
    Description: This function will find the color of the sample
    Inputs: 
    Outputs: Color of sample
    """
    image_path = record_image()
    image_rb = remove_background_fs(image_path)
    return dominant_color(image_rb)

def plot_color_change(color_change, times):
    fig = plt.figure(figsize=(5, 2))
    ax = fig.add_subplot(111)
    for i, color in enumerate(color_change):
        color_norm = color/255
        color_rect = matplotlib.patches.Rectangle((times[i], 0), (times[i+1]-times[i]), 2, color=color_norm)
        ax.add_patch(color_rect)

    plt.xlim([times[0], times[-1]])
    plt.ylim([0, 1])
    plt.tick_params(left = False, right = False , labelleft = False)
    plt.title("Color change of Sample Over Time")
    plt.xlabel("Time (s)")
    plt.show()

def foam_height(path):
    """
    Title: Finds foam height
    Description: This function will measure the height of foam
    Inputs: Image
    Outputs: Foam height
    """
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
    rectangle = (255, 328, 215, 55)

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
    
    # Grab the alpha pixels above a specified threshold
    alpha_threshold = 190
    mask_arr = np.array(np.greater(np.array(tmp), alpha_threshold), dtype=np.uint8)
    hard_mask = Image.fromarray(np.uint8(mask_arr) * 255, 'L')
    
    # Get the smallest & largest non-zero values in y
    nz = np.nonzero(hard_mask)
    height = abs(np.min(nz[0]) - np.max(nz[0])) # miny minx maxy maxx
    return height * 0.33 # scaling factor from pixel -> mm


print(foam_height("ProjectPics\Light_LS1HCover.jpg"))
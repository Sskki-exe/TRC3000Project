def remove_background():
    mp_selfie_segmentation = mp.solutions.selfie_segmentation
    selfie_segmentation = mp_selfie_segmentation.SelfieSegmentation(model_selection=1)
    
    frame = record_image()
    cv2.imshow("out",frame)
    cv2.waitKey(0)
    RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # get the result
    results = selfie_segmentation.process(RGB)
    # extract segmented mask
    mask = results.segmentation_mask
    # it returns true or false where the condition applies in the mask
    condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.5
    src = condition * frame

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
    
    # Writing and saving to a new image
    cv2.imshow("out",dst)
    cv2.waitKey(0)
    cv2.imwrite("gfg_white.png", dst)

# path to input image specified and
# image is loaded with imread command
image = cv2.imread("ProjectPics/testImage.jpg")

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
rectangle = (50, 250, 150, 100)

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
cv2.imshow("image", dst)
cv2.waitKey(0)
dominant_color(dst)

# def detect_bottle(path):
#     bottle_3_channel = cv2.imread(path)
#     bottle_gray = cv2.cvtColor(bottle_3_channel, cv2.COLOR_BGR2GRAY)
#     bottle_gray = cv2.GaussianBlur(bottle_gray, (7, 7), 0)
#     (T, bottle_threshold) = cv2.threshold(bottle_gray, 25, 255, cv2.THRESH_BINARY_INV)
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#     bottle_open = cv2.morphologyEx(bottle_threshold, cv2.MORPH_OPEN, kernel)
#     #cv2.imshow("All Contours", bottle_gray)
#     contours = cv2.findContours(bottle_open.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     contours = imutils.grab_contours(contours)
#     bottle_clone = bottle_3_channel.copy()
#     cv2.drawContours(bottle_clone, contours, -1, (255, 0, 0), 2)
#     cv2.imshow("All Contours", bottle_clone)
#     cv2.waitKey(0)
    
# detect_bottle("./ProjectPics/im22.jpg")
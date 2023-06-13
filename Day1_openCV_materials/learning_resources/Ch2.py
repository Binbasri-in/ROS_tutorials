import cv2 
import numpy as np

# Some imp openCV functions

img=cv2.imread("Resources\lena.png")

# Converting to grayscale

imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # BGR to GRAY...BGR is default color scheme of openCV
# cv2.imshow("Gray Image",imgGray)
# cv2.waitKey(0)

#                                       Blurring

imgBlur=cv2.GaussianBlur(imgGray,(7,7),0) # (7,7) is kernel size and 0 is sigmaX
# kerenel should be odd and greater than 1
# sigmaX is standard deviation in X direction
# kernel is used to take average of surrounding pixels
# sigmaX is used to calculate weight of each pixel
# cv2.imshow("Blurred Image",imgBlur)
# cv2.waitKey(0)

#                                       Edge Detector

imgCanny=cv2.Canny(img,100,200) 
# threshold values are used to detect edges..how it works is that it takes gradient of image and if gradient is greater than threshold value then it is an edge
# 2 parameters of canny are threshold values one is lower and other is upper
# if gradient is greater than upper threshold value then it is an edge
# if gradient is less than lower threshold value then it is not an edge
# if gradient is between lower and upper threshold value then it is an edge if it is connected to an edge

# cv2.imshow("Canny Image",imgCanny)
# cv2.waitKey(0)

#                                           Dilating the image

# The process of increasing thickness of edges is called dialation
# Dilation is done to remove noise from image
# Because sometimes edges are not connected and we want to connect them so we dialate the image to get connected edges
kernel=np.ones((5,5),np.uint8) # kernel is a matrix of 1s

imgDialation=cv2.dilate(imgCanny,kernel,iterations=1) # iterations is how many times we want to dialate the image
# cv2.imshow("Dialated Image",imgDialation)
# cv2.waitKey(0)

# Eroding the image

# The process of decreasing thickness of edges is called erosion
# Erosion is done to remove noise from image
# Eroison is opposite of dialation...it is done to remove extra edges from image and to make edges thinner
# it is usually done after dialation

imgEroded=cv2.erode(imgDialation,kernel,iterations=1)
# cv2.imshow("Eroded Image",imgEroded)
# cv2.waitKey(0)



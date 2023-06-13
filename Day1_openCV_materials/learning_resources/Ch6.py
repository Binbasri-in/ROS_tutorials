import cv2 
import numpy as np

# horizontally stacking images and vertically stacking images

img=cv2.imread("Resources\lena.png")
imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # BGR to GRAY...BGR is default color scheme of openCV

imgHor=np.hstack((img,img)) # horizontally stacking images

imgVer=np.vstack((img,img)) # vertically stacking images

cv2.imshow("Horizontal Image",imgHor)
cv2.imshow("Vertical Image",imgVer)
cv2.waitKey(0)
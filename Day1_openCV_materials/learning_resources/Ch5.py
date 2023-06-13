import cv2 
import numpy as np

# Warp Perspective
# It is used to get bird eye view of image

img=cv2.imread("Resources\Cards.png")
width,height=250,350
pts1=np.float32([[111,219],[305,188],[154,500],[370,458]]) # 4 points of card
pts2=np.float32([[0,0],[width,0],[0,height],[width,height]]) # 4 points of new image
matrix=cv2.getPerspectiveTransform(pts1,pts2) # matrix is used to warp perspective
imgOutput=cv2.warpPerspective(img,matrix,(width,height)) # (image,matrix,(width,height))
cv2.imshow("Image",img)
cv2.imshow("Output",imgOutput)
cv2.waitKey(0)

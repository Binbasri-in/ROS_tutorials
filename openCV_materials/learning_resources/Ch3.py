import cv2 
import numpy as np

# Resizing 

img=cv2.imread("Resources\chess.jpg")
print(img.shape) # (height,width,channels)
imgResize=cv2.resize(img,(300,200)) # (width,height)
print(imgResize.shape)
cv2.imshow("Original Image",img)
cv2.imshow("Resized Image",imgResize)
cv2.waitKey(0)

# Cropping

imgCropped=img[0:200,200:500] # (height,width) NOTE: in cropping we have to give height first and then width
cv2.imshow("Cropped Image",imgCropped)  
cv2.waitKey(0)
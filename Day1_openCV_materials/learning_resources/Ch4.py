import cv2 
import numpy as np

# Shapes and Texts

img=np.zeros((512,512,3),np.uint8) # (height,width,channels) NOTE: height and width are in reverse order
print(img.shape)
# img[:]=255,0,0 # (B,G,R) NOTE: BGR is default color scheme of openCV
# cv2.imshow("Image",img)
# img[200:300,100:300]=255,0,0 # (B,G,R) NOTE: BGR is default color scheme of openCV
# cv2.imshow("Image_2",img)
# cv2.line(img,(0,0),(512,512),(0,255,0),3) # (image,starting point,ending point,color,thickness)
# cv2.imshow("Image_3",img)
# rectangle 
# cv2.rectangle(img,(0,0),(250,350),(0,0,255),cv2.FILLED) # (image,starting point,ending point,color,thickness)
# cv2.imshow("Image_4",img)
# circle
# cv2.circle(img,(400,50),30,(255,255,0),5) # (image,center,radius,color,thickness)
# cv2.imshow("Image_5",img)
# text
cv2.putText(img,"OpenCV",(300,200),cv2.FONT_HERSHEY_COMPLEX,1,(0,150,0),1) # (image,text,starting point,font,font scale,color,thickness)
cv2.imshow("Image_6",img)

cv2.waitKey(0)
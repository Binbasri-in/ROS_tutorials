import cv2 
import numpy as np

# Color detection

def empty(a):
    pass

# Color we need to detect is Lamborghini's color

# Colour we need=White
# colour we don't need=Black
# Hue=0-179 in openCV (0-360 in real world)  
# Hue is the color
# Saturation is the purity of the color
# Value is the brightness of the color

path="Resources\lambo.png"
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty) # 0 is the default value
cv2.createTrackbar("Hue Max","TrackBars",179,179,empty) # 179 is the default value
cv2.createTrackbar("Sat Min","TrackBars",0,255,empty) # 0 is the default value
cv2.createTrackbar("Sat Max","TrackBars",255,255,empty) # 255 is the default value
cv2.createTrackbar("Val Min","TrackBars",0,255,empty) # 0 is the default value
cv2.createTrackbar("Val Max","TrackBars",255,255,empty) # 255 is the default value

while True:
    img=cv2.imread(path)
    imgHSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV) # BGR to HSV...HSV is default color scheme of openCV
    h_min=cv2.getTrackbarPos("Hue Min","TrackBars")
    h_max=cv2.getTrackbarPos("Hue Max","TrackBars")
    s_min=cv2.getTrackbarPos("Sat Min","TrackBars")
    s_max=cv2.getTrackbarPos("Sat Max","TrackBars")
    v_min=cv2.getTrackbarPos("Val Min","TrackBars")
    v_max=cv2.getTrackbarPos("Val Max","TrackBars")
    print(h_min,h_max,s_min,s_max,v_min,v_max)
    lower=np.array([h_min,s_min,v_min])
    upper=np.array([h_max,s_max,v_max])
    mask=cv2.inRange(imgHSV,lower,upper) # mask is the filtered image
    imgResult=cv2.bitwise_and(img,img,mask=mask) # imgResult is the final image
    cv2.imshow("Original Image",img)
    cv2.imshow("HSV Image",imgHSV)
    cv2.imshow("Mask Image",mask)
    cv2.imshow("Result Image",imgResult)
    cv2.waitKey(1)




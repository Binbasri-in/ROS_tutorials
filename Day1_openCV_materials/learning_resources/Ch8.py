import cv2 
import numpy as np

# Contours and shape detection

img=cv2.imread('Resources\shapes.png')
imgContour=img.copy()

def getContours(img):
    contours, hierarchy =cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) # RETR_EXTERNAL: retrieves the extreme outer contours
    # CHAIN_APPROX_NONE is an approximation method that retains all the contour points without any compression or simplification. It means that the contour is not approximated or smoothed in any way, and all the original points are preserved. This can be useful in situations where you need precise and detailed information about the contour, such as when performing high-precision shape analysis, contour matching, or calculating precise measurements based on the contour.
    # However, using CHAIN_APPROX_NONE can result in a large number of contour points, which can be memory-intensive and may slow down subsequent processing steps. Therefore, it is generally recommended to use contour approximation methods like CHAIN_APPROX_SIMPLE or CHAIN_APPROX_TC89_L1 instead.
    for cnt in contours:
        area=cv2.contourArea(cnt) # Contour area
        print(area)
        if area>200:
            cv2.drawContours(imgContour,cnt,-1,(255,0,0),3)# -1: draw all contours, 3: thickness
            peri=cv2.arcLength(cnt,True)# True: closed contour
            print(peri)# Perimeter
            approx=cv2.approxPolyDP(cnt,0.02*peri,True)# Approximation..to get the number of corners
            print(len(approx))# Number of corners
            objCor=len(approx)# Number of corners
            x,y,w,h=cv2.boundingRect(approx)# Bounding box
            if objCor==3: objectType="Tri"# Object type
            elif objCor==4:
                aspRatio=w/float(h)
                if aspRatio>0.95 and aspRatio<1.05: objectType="Square"
                else: objectType="Rectangle"
            elif objCor==5: objectType="Pentagon"
            elif objCor==6: objectType="Hexagon"
            elif objCor==7: objectType="Heptagon"
            elif objCor>8: objectType="Circle"
            else: objectType="Circle"
            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),2)# Draw the bounding box
            cv2.putText(imgContour,objectType,(x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,0),2)# Put the object type on the image

imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)# Convert to gray
imgBlur=cv2.GaussianBlur(imgGray,(7,7),1)# Blur
imgCanny=cv2.Canny(imgBlur,50,50)# Canny edge detection
getContours(imgCanny)   # Call the function

cv2.imshow("Original",img)
cv2.imshow("Gray",imgGray)
cv2.imshow("Blur",imgBlur)
cv2.imshow("Canny",imgCanny)
cv2.imshow("Contour",imgContour)
cv2.waitKey(0)
cv2.destroyAllWindows()






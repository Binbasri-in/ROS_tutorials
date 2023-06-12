import cv2 

# Face detection using Haar cascades
faceCascade=cv2.CascadeClassifier("Resources\HaarCascade_frontal_face_default.xml")
# img=cv2.imread("Resources\lena.png")
img=cv2.imread("Resources\Faces.png")
imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
faces=faceCascade.detectMultiScale(imgGray,1.1,4)
for (x,y,w,h) in faces:
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
cv2.imshow("Result",img)
cv2.waitKey(0)
cv2.destroyAllWindows()

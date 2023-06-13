import cv2 

# IMAGE INPUT

img=cv2.imread("Resources\lena.png")
cv2.imshow("Window",img)
cv2.waitKey(0)

# VIDEO INPUT

# video=cv2.VideoCapture("Resources\\testVideo.mp4")

# while True:
#     success,img=video.read()
#     cv2.imshow("Video",img)
#     if cv2.waitKey(1) & 0xFF==ord('q'):
#         break

# WEBCAM INPUT

# video=cv2.VideoCapture(0)
# video.set(3,640)  # 3 is id of width 
# video.set(4,480)  # 4 is id of height
# video.set(10,100) # 10 is id of brightnessq

# while True:
#     success,img=video.read()
#     cv2.imshow("Video",img)
#     if cv2.waitKey(1) & 0xFF==ord('q'):
#         break
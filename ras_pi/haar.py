import numpy as np
import cv2
import time
from pathlib import Path

cascade_path = Path("/home/pi/serial_interface/capstone-quadcopter/ras_pi/haarcascade_frontalface_default.xml")
print(cascade_path)

face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')
casc = face_cascade.empty()
if (casc):
    print("no cascade filter found")
cap = cv2.VideoCapture(0)
    
if not cap.isOpened():
    print("unable to open camera")
    
while(cap.isOpened() and (not casc)):
    start_cap = time.time()
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    end_cap = time.time()
    delta_cap = (end_cap - start_cap) * 1000
    print("%.2f ms to capture a pic" %delta_cap)

    start_calc = time.time()
    # first val in next function is scale factor, lower val means you can go further away
    # second val is number of bounding boxes that must intersect to count as a face
    # largr vals are better for up close, smaller valsfor further away
    faces, num_faces = face_cascade.detectMultiScale2(gray, 1.4, 2)
    for (x,y,w,h) in faces:
        img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]

    end_calc = time.time()
    delta_calc = (end_calc - start_calc) * 1000
    print("%.2f ms to calculate" %delta_calc)
    
    start_show = time.time()
    cv2.imshow('img', img)
    end_show = time.time()
    delta_show = (end_show - start_show) * 1000
    print("%.2f ms to show image" %delta_show)
    #cv2.waitKey(0)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

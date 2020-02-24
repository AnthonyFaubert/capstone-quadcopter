import numpy as np
import cv2
import time
import code
import socket
import imutils

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('localhost', 8000))
s.listen(1)
conn, addr = s.accept() #use conn.recv(), conn.send()

face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml') #filter can be swapped with ....face_alt.xml
no_face_frames = 0
#prev_center = [319,119] #arbitrary values that correspond to center of frame
#prev_size = [100, 100] #arbitrary values for person sitting about 1 foot in front of camera

if (face_cascade.empty()):
    print("No cascade filter found.")

cap = cv2.VideoCapture(0)
faces = []
init_frames = 0
    
if not cap.isOpened():
    print("unable to open camera")
else:
    while(len(faces) == 0):
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces, num_faces = face_cascade.detectMultiScale2(gray, 1.8, 3)
        #code.interact(local=locals())
        init_frames = init_frames + 1

print("The total number of intialization frames before a face was found was " + str(init_frames))

prev_center = [faces[0][0]+(0.5*faces[0][2]), faces[0][1]+(0.5*faces[0][3])]
prev_size = [faces[0][2], faces[0][3]]
    
while(cap.isOpened() and (not face_cascade.empty())):
    start_cap = time.time()
    ret, img = cap.read()
    angle = 0 # need to change this to socket.recv
    rot = imutils.rotate(img, angle)
    gray = cv2.cvtColor(rot, cv2.COLOR_BGR2GRAY)
    end_cap = time.time()
    delta_cap = (end_cap - start_cap) * 1000
    print("%.2f ms to capture a pic" %delta_cap)

    start_calc = time.time()
    # first val in next function is scale factor, lower val means you can go further away
    # second val is number of bounding boxes that must intersect to count as a face
    # largr vals are better for up close, smaller valsfor further away
    faces, num_faces = face_cascade.detectMultiScale2(gray, 1.5, 3)
    face_det = False
    for (x,y,w,h) in faces:
        center = [x + (0.5*w), y + (0.5*h)]
        size = [w, h]
        img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        print('x of bounding: ' + str(x))
        print('y of bounding: ' + str(y))
        print('width: ' + str(w))
        print('height: ' + str(h))
        #change checks below to abs val
        if (center[0] < (prev_center[0]+50) and center[0] > (prev_center[0]-50)
            and center[1] < (prev_center[1]+50) and center[1] > (prev_center[1]-50)
            and size[0] < (prev_size[0]*2) and size[0] > (prev_size[0]*0.5)
            and size[1] < (prev_size[1]*2) and size[1] > (prev_size[1]*0.5)):
            prev_center = [x + (0.5*w), y + (0.5*h)]
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
            prev_size = [w, h]
        face_det = True

    x_err = prev_center[0] - 239
    y_err = prev_center[1] - 179

    if (not face_det):
        no_face_frames = no_face_frames + 1
        print("Number of frames with no face is " + str(no_face_frames) + ".")

    end_calc = time.time()
    delta_calc = (end_calc - start_calc) * 1000
    print("%.2f ms to calculate" %delta_calc)
    
    start_show = time.time()
    cv2.imshow('img', img)
    end_show = time.time()
    delta_show = (end_show - start_show) * 1000
    print("%.2f ms to show image" %delta_show)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("total initialization frames = " + str(init_frames))

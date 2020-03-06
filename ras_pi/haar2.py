import numpy as np
import cv2
import time
import datetime
import code
import socket
import imutils
import struct

use_socket = False
# establishing socket connection
if (use_socket):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', 8000))
    s.listen(1)
    conn, addr = s.accept() #use conn.recv(), conn.send()
    print("Connection established!")
    conn.setblocking(0)

face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml') #filter can be swapped with ....face_alt.xml

if (face_cascade.empty()):
    print("ERROR: No cascade filter found.")
    quit()

cap = cv2.VideoCapture(0)
   
if not cap.isOpened():
    print("ERROR: Unable to open camera")
    quit()

consec_missed = 20 # this value is used so it searches the whole image for a face
no_face_frames = 0
prev_center = [319, 179]
prev_size = 100
loop_time = -1

# main program loop
while(cap.isOpened() and (not face_cascade.empty())):
    start_cap = time.time()
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    end_cap = time.time()
    delta_cap = (end_cap - start_cap) * 1000
    print("%.2f ms to capture a pic" %delta_cap)

    start_calc = time.time()
    # first val in next function is scale factor, lower val means you can go further away
    # second val is number of bounding boxes that must intersect to count as a face
    # larger vals are better for up close, smaller valsfor further away
    faces = face_cascade.detectMultiScale(gray, 1.8, 3)
    face_det = False
    for (x,y,w,h) in faces:
        face_det = True
        center = [x + (0.5*w), y + (0.5*h)]
        size = [w, h]
        img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
        # check to see if this face is the same as face from last frame
        if (abs(center[0] - prev_center[0]) < (30*(consec_missed+1))
            and abs(center[1] - prev_center[1]) < (30*(consec_missed+1))
            and size[0] < (prev_size[0]*2) and size[0] > (prev_size[0]*0.5)
            and size[1] < (prev_size[1]*2) and size[1] > (prev_size[1]*0.5)):
            if(loop_time == -1):
                d_time = 0.005
            else:
                d_time = time.time() - loop_time
            x_diff = (center[0] - prev_center[0])/d_time
            y_diff = (center[1] - prev_center[1])/d_time
            w_diff = (size[0] - prev_size[0])/d_time
            loop_time = time.time()
            prev_center = [x + (0.5*w), y + (0.5*h)]
            prev_size = w
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

    if (use_socket):
        rec_bytes = conn.recv(1)
        if(len(rec_bytes) == 1):
            d = datetime.datetime.now()
            image_file_path = "/var/www/html/flight_pics/pic_%04d-%02d-%02d_%02d.%02d.%02d.png" % (d.year, d.month, d.day, d.hour, d.minute, d.second)
            cv2.imwrite(image_file_path, img)

    x_err = prev_center[0] - 239
    y_err = prev_center[1] - 179
    
    if (face_det):
        consec_missed = 0
    else: #not face_det
        no_face_frames = no_face_frames + 1
        consec_missed += 1
        
    if (use_socket):
        send_data = struct.pack('>hhhhhh', x_err, x_diff, y_err, y_diff, prev_size, w_diff)
            conn.send(send_data)

    if (consec_missed > 1000):
        consec_missed = 20
    
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

s.close()

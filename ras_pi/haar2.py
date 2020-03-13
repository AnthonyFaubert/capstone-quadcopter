# This script must be run before 'controller.py' because it acts as the server side
# of the port connection.
# The script handles the image capture, image processing, and saving of the
# image. The image processing consists of using a haar cascade filter to
# find faces within the image frame. It then sends the x and y errors
# across the socket connection to 'controller.py' to be used in the controls.

# created by Griffin Hardy, Tony Faubert, Lisa Qing, and David Joslin for
# EE/CSE 475 Emebdded Systems Capstone, Winter Quarter 2020

import numpy as np
import cv2
import time
import datetime
import code
import socket
import imutils
import struct

from picamera import PiCamera
from picamera.array import PiRGBArray
cam = PiCamera()
cam.rotation = 90
cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray(cam, size=cam.resolution)
time.sleep(0.1) # "allow the camera to warmup"

# variable to determine if we want to run it on its own or with controller.py
use_socket = True
# establishing socket connection
if (use_socket):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', 8000))
    s.listen(1)
    conn, addr = s.accept()
    print("Connection established!")
    conn.setblocking(0)

# load the cascade filter into a variable
face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml') #filter can be swapped with ....face_alt.xml

# check to make sure the filter loaded properly
if (face_cascade.empty()):
    print("ERROR: No cascade filter found.")
    quit()

cap = cv2.VideoCapture(0)

# make sure the camera is open
if not cap.isOpened():
    print("ERROR: Unable to open camera")
    quit()

# initialize variable starting values
consec_missed = 20 # this value is used so it searches the whole image for a face upon startup
no_face_frames = 0
prev_center = [319, 179]
prev_size = 100
loop_time = -1
x_diff = 0
y_diff = 0
w_diff = 0

# main program loop
for frame in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    start_cap = time.time()
    img = frame.array
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
        size = w
        img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
        # below is the check to see if this face is the same as face from last frame
        if (abs(center[0] - prev_center[0]) < (100*(consec_missed+1))
            and abs(center[1] - prev_center[1]) < (100*(consec_missed+1))
            and size < (prev_size*3) and size > (prev_size*0.33)):
            if(loop_time == -1):
                d_time = 0.005
            else:
                d_time = time.time() - loop_time
            x_diff = (center[0] - prev_center[0])/(d_time)
            y_diff = (center[1] - prev_center[1])/d_time
            w_diff = (size - prev_size)/d_time
            loop_time = time.time()
            prev_center = [x + (0.5*w), y + (0.5*h)]
            prev_size = w
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

    if (use_socket):
        try:
            rec_bytes = conn.recv(1) # read the next byte from receive buffer
        except BlockingIOError as e:
            rec_bytes = b''
        if(len(rec_bytes) == 1): # check to see if we received any data through the socket
            d = datetime.datetime.now()
            image_file_path = "/var/www/html/flight_pics/pic_%04d-%02d-%02d_%02d.%02d.%02d.png" % (d.year, d.month, d.day, d.hour, d.minute, d.second)
            cv2.imwrite(image_file_path, img)

    # calculate the errors between the face center and center of the image
    x_err = prev_center[0] - 239
    y_err = prev_center[1] - 179
    
    if (face_det):
        consec_missed = 0
    else: #not face_det
        no_face_frames = no_face_frames + 1
        consec_missed += 1
        
    if (use_socket):
        if (x_diff > 32767):
            x_diff = 32767
        if (x_diff < -32767):
            x_diff = -32767
        if (y_diff > 32767):
            y_diff = 32767
        if (y_diff < -32767):
            y_diff = -32767
        if (w_diff > 32767):
            w_diff = 32767
        if (w_diff < -32767):
            w_diff = -32767
send_data = struct.pack('>hhhhhh', int(x_err), int(x_diff), int(y_err), int(y_diff), int(prev_size), int(w_diff))
    conn.send(send_data) # send data to controller.py

    # reset if consecutive missed frames gets too large
    if (consec_missed > 1000):
        consec_missed = 20 # this value still searches the entire frame
    
    end_calc = time.time()
    delta_calc = (end_calc - start_calc) * 1000
    print("%.2f ms to calculate" %delta_calc)
    
    start_show = time.time()
    cv2.imshow('img', img)
    end_show = time.time()
    delta_show = (end_show - start_show) * 1000
    print("%.2f ms to show image" %delta_show)

    rawCapture.truncate(0) # clear the stream to get the actual next frame
    if cv2.waitKey(1) & 0xFF == ord('q'): # if the 'Q' key was pressed
        break

# close the socket connection
conn.close()
s.close()

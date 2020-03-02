#!/usr/bin/python3

import pygame
import time
import datetime
import serial
import subprocess
import struct
import socket
import code

UART_BAUD_RATE = 460800
LOOP_RATE = 80 # Hz
DATA_LENGTH = 12
manual = True
use_socket = False
# establishing the socket connection
if (use_socket):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 8000))
    print("connection established!")
    s.setblocking(0)

# intialize pygame
pygame.init()
pygame.joystick.init()

#screen = pygame.display.set_mode((100, 100))
#pygame.display.set_caption("My Game")
clock = pygame.time.Clock()

# initialize the joystick object that is the Steam controller
joystick = pygame.joystick.Joystick(0)
joystick.init() 

# set up file path for our flight log
d = datetime.datetime.now()
debug_str = "/var/www/html/flight_logs/debug_%04d%02d%02d_%02d%02d%02d.txt" % (d.year, d.month, d.day, d.hour, d.minute, d.second)
debug_file = open(debug_str,'wb')

# initializes UART with non-blocking read and write
ser = serial.Serial(port="/dev/ttyS0", baudrate=UART_BAUD_RATE, write_timeout=None, timeout=None)
special = 0

def hexify(bs):
    return ''.join('{:02x}'.format(x) for x in bs)

def get_tilt(): # left analog stick, axes 0 and 1. Axis 1 is up/down and is inverted with fully up being -1
    lr_motion = int(joystick.get_axis(0) * 32767)
    ud_motion = int(joystick.get_axis(1) * -32767)
    return (lr_motion, ud_motion)

def get_yt(): # right trackpad used as stick, axes 3 and 4. Axis 4 is up/down and is inverted, fully up is -1
    lr_motion = int(joystick.get_axis(3) * 32767)
    ud_motion = int(joystick.get_axis(4) * -32767)
    return (lr_motion, ud_motion)

def connection():
    if (pygame.joystick.get_count() == 0):
        return False
    else:
        return True
    
ser.reset_input_buffer()
ser.reset_output_buffer()
lastSend = -1
worstLoopTime = -1
cont_freeze_frames = -1
done = False

# main program loop
while not done:
    clock.tick(LOOP_RATE) #integer value is the frames per second
    loopTime = time.time()
    debug_file.flush()
    
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            if (event.button == 6 or event.button == 7):
                special = 5
            elif (event.button == 0):
                special = 1
            elif (event.button == 1):
                special = 2
            elif (event.button == 2):
                special = 3
            elif (event.button == 3):
                special = 4

    #receive data from haar.py via socket for face position in frame
    if (use_socket):
        face_data_bytes = face_data_extra + s.recv(1000)
        face_data_extra_num = len(face_data_bytes) % 12
        face_data_extra = face_data_bytes[((len(face_data_bytes)) - face_data_extra_num):]
        face_data_start = len(face_data_bytes) - 12 - face_data_extra_num
        face_data_end = len(face_data_bytes) - face_data_extra_num
        final_face_bytes = face_data_bytes[face_data_start:face_data_end]
        # face_data is in format x_err, x_derivative, y_err, y_derivative, width, width_derivative
        face_data = struct.unpack('>hhhhhh', final_face_bytes)

    # get controller joystick values
    tilt = get_tilt()
    yt = get_yt()
    # initialization loop
    if (cont_freeze_frames == -1):
        prev_tilt = tilt
        prev_yt = yt

    if (prev_tilt == tilt and prev_yt == yt and tilt != (0,0) and yt != (0,0)):
        cont_freeze_frames += 1
        if (cont_freeze_frames > 10):
            special = 5
    else:
        cont_freeze_frames = 0

    if (manual):
        data = struct.pack('>Bhhhhh', 37, tilt[0], tilt[1], yt[0], yt[1], special)
    else:
        #data is x_err, x_derivative, y_err, y_derivative, width, width_derivative
        #data = struct.pack('>Bhhhhh', 37, face_data[0], face_dat width, 0, face_data y, special)

    checksum = sum(data) & 0xFF
    data += struct.pack('>B', checksum)
    print("below is sent data:")
    print(hexify(data))
    ser.write(data)
    if (special == 5 or cont_freeze):
        # send multiple E-stops to ensure reception
        ser.write(data)
        ser.write(data)
        #done = True
        time.sleep(3) # give time to receive all debug UART data before ending
    timeDiff = time.time() - lastSend
    if (lastSend != -1) and (timeDiff > 0.030):
        loopTime = time.time() - loopTime
        print("ERROR: %f seconds in between packet sends!" % timeDiff)
        print("This loop time: %f. Worst loop time: %f." % (loopTime, worstLoopTime))
            #time.sleep(15)
    lastSend = time.time()
    print()
    special = 0

    if (use_socket):
        # receive data from UART for angle of face
        angle_rec = ser.read(ser.in_waiting)
        # send data to haar.py
        s.send(angle_rec[-1])

    if (ser.in_waiting > 0):
        ser.ready_to_send = True
        debug_file.write(ser.read(ser.in_waiting))
    loopTime = time.time() - loopTime
    if loopTime > worstLoopTime:
        worstLoopTime = loopTime

#turn off motors
ser.close()
s.close()
pygame.quit()
debug_file.close()

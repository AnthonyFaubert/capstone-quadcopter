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
use_socket = True
# establishing the socket connection
if (use_socket):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('localhost', 8000))
        print("connection established!")
        s.setblocking(0)
    except:
        use_socket = False
        print("Could not find socket")

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

# hold values when the user lets go
_get_yt_last_ud_motion = 0
_get_yt_last_lr_motion = 0
def get_yt(): # right trackpad used as stick, axes 3 and 4. Axis 4 is up/down and is inverted, fully up is -1
    global _get_yt_last_ud_motion, _get_yt_last_lr_motion
    lr_motion = int(joystick.get_axis(3) * 32767)
    ud_motion = int(joystick.get_axis(4) * -32767)
    if (ud_motion == 0) and (lr_motion == 0):
        ud_motion = _get_yt_last_ud_motion
        lr_motion = _get_yt_last_lr_motion
    else:
        _get_yt_last_ud_motion = ud_motion
        _get_yt_last_lr_motion = lr_motion
    return (lr_motion, ud_motion)

def connection():
    if (pygame.joystick.get_count() == 0):
        return False
    else:
        return True

def send_button(butt):
    data = struct.pack('>BhhhhhB',0x25, 0, 0, 0, 0, butt, (0x25+butt))
    ser.write(data)
    if (butt == 5):
        ser.write(data)
        ser.write(data)
        print('Sent e-stop!')
        time.sleep(3)

def calc_auto_js(x_err, x_der, y_err, y_der, w_err, w_der):
    return (0,0,0,0,0)
    
ser.reset_input_buffer()
ser.reset_output_buffer()
lastSend = -1
worstLoopTime = -1
cont_freeze_frames = -1
last_change_timestamp = -1
prev_auto = -1
axis2init = False
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
            if (event.button == 5):
                send_button(5) # e-stop
            elif (event.button == 0):
                special = 1
            elif (event.button == 1):
                special = 2
            elif (event.button == 2):
                special = 3
            elif (event.button == 3):
                special = 4
            elif (event.button == 4):
                if (use_socket):
                    sock_send = struct.pack('>b', 4)
                    s.send(sock_send)
                else:
                    print('cannot take picture, not connected to haar.py')
                pass
            elif (event.button == 6):
                special = 6 # reset flight log
                pass
            elif (event.button == 7):
                send_button(7) # start quad motors
                pass
    

    if(last_change_timestamp == -1):
        last_change_timestamp = time.time()

    if(not axis2init and joystick.get_axis(2)==0.0):
        auto = -32767
    else:
        auto = int(joystick.get_axis(2) * 32767)
        axis2init = True

    if (auto == -32767):
        manual = True
    elif (prev_auto != auto):
        last_change_timestamp = time.time()
        prev_auto = auto
        manual = False
    if (not manual and (time.time() - last_change_timestamp) > 1):
            send_button(5)

    #receive data from haar.py via socket for face position in frame
    if (use_socket):
        face_data_bytes = face_data_extra + s.recv(1000)
        face_data_extra_num = len(face_data_bytes) % 12
        face_data_extra = face_data_bytes[((len(face_data_bytes)) - face_data_extra_num):]
        face_data_start = len(face_data_bytes) - 12 - face_data_extra_num
        face_data_end = len(face_data_bytes) - face_data_extra_num
        final_face_bytes = face_data_bytes[face_data_start:face_data_end]
        # face_data is in format x_err, x_derivative, y_err, y_derivative, width, width_derivative
        [x_err, x_der, y_err, y_der, w_err, w_der] = struct.unpack('>hhhhhh', final_face_bytes)

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
            send_button(5)
    else:
        cont_freeze_frames = 0

    if (manual or not use_socket): # condition should be set to 'manual'
        data = struct.pack('>Bhhhhh', 37, tilt[0], tilt[1], yt[0], yt[1], special)
    else:
        auto_joystick = calc_auto_js(x_err, x_der, y_err, y_der, w_err, w_der)
        #data is x_err, x_derivative, y_err, y_derivative, width, width_derivative
        if (special == 0):
            data = struct.pack('>Bhhhhh', 37, *auto_joystick)
        else:
            data = struct.pack('>Bhhhhh', 37, *auto_joystick[:-1], special)

    checksum = sum(data) & 0xFF
    data += struct.pack('>B', checksum)
    print("below is sent data:")
    print(hexify(data))
    ser.write(data)

    timeDiff = time.time() - lastSend
    if (lastSend != -1) and (timeDiff > 0.050):
        loopTime = time.time() - loopTime
        print("ERROR: %f seconds in between packet sends!" % timeDiff)
        print("This loop time: %f. Worst loop time: %f." % (loopTime, worstLoopTime))
        time.sleep(15)
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

    print('auto value:%d' %auto)

#turn off motors
ser.close()
s.close()
pygame.quit()
debug_file.close()

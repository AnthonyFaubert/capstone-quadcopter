#!/usr/bin/python3

import pygame
import time
import datetime
import serial
import subprocess
import struct
import socket
import code

#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect(('localhost', 8000))
print("connection established!")
#s.setblocking(False)

manual = True
auto = False

DATA_LENGTH = 12

# intialize pygame
pygame.init()
pygame.joystick.init()

done = False
ready_to_send = True
d = datetime.datetime.now()
#screen = pygame.display.set_mode((100, 100))
#pygame.display.set_caption("My Game")
clock = pygame.time.Clock()

joystick = pygame.joystick.Joystick(0)
joystick.init()

debug_str = "/var/www/html/debug_" + str(d.year) + str(d.month) + str(d.day) + "_" + str(d.hour) + "-" + str(d.minute) + ".txt"
debug_file = open(debug_str,'wb')

ser = serial.Serial(port = "/dev/ttyS0", baudrate = 115200, write_timeout = None, timeout = None) #read timeout should get set to None
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
while not done:
    clock.tick(70) #integer value is the frames per second
    loopTime = time.time()
    debug_file.flush()
    
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            if (event.button == 6 or event.button == 7):
                special = 5
                print("button 5")
            elif (event.button == 0):
                special = 1
                print("button 1")
            elif (event.button == 1):
                special = 2
                print("button 2")
            elif (event.button == 2):
                special = 3
                print("button 3")
            elif (event.button == 3):
                special = 4
                print("button 4")

    tilt = get_tilt()
    yt = get_yt()
    ready_to_send = True  #needs to be set to False eventually
    #face_data_bytes = face_data_extra + s.recv(1000)
    #face_data_extra_num = len(face_data_bytes) % 12
    #face_data_extra = face_data_bytes[((len(face_data_bytes)) - face_data_extra_num):]
    #face_data_start = len(face_data_bytes) - 12 - face_data_extra_num
    #face_data_end = len(face_data_bytes) - face_data_extra_num
    #final_face_bytes = face_data_bytes[face_data_start:face_data_end]
    #face_data = struct.unpack('>hhhhhh', final_face_bytes)
        
        if (manual):
            data = struct.pack('>Bhhhhh', 37, tilt[0], tilt[1], yt[0], yt[1], special)
        #else:
        #data = struct.pack('>Bhhhhh', 37, face_data x lr, face_dat width, 0, face_data y, special)
    checksum = sum(data) & 0xFF
    data += struct.pack('>B', checksum)
    print("below is sent data:")
    ser.write(data)
    if special == 5:
            # send multiple E-stops to ensure reception
        ser.write(data)
        ser.write(data)
        #done = True
        time.sleep(3) # give time to receive all debug UART data before ending
    print(hexify(data))
    timeDiff = time.time() - lastSend
    if (lastSend != -1) and (timeDiff > 0.030):
        loopTime = time.time() - loopTime
        print("ERROR: %f seconds in between packet sends!" % timeDiff)
        print("This loop time: %f. Worst loop time: %f." % (loopTime, worstLoopTime))
            #time.sleep(15)
    lastSend = time.time()
    print()
    special = 0

    #send angle to haar.py
    #angle = struct.pack('>b', )
    #s.send(angle)


    if (ser.in_waiting > 0):
        #print("below is bytes waiting")
        #print(ser.in_waiting)
        ser.ready_to_send = True
        debug_file.write(ser.read(ser.in_waiting))
        #ser.reset_input_buffer()
    loopTime = time.time() - loopTime
    if loopTime > worstLoopTime:
        worstLoopTime = loopTime

#turn off motors
ser.close()
pygame.quit()
debug_file.close()

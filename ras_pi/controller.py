#!/usr/bin/python3

import pygame
import time
import serial
import subprocess
import struct
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s = s.connect(('localhost', 8000))

DATA_LENGTH = 12

# open the steam controller driver
#sc_proc = subprocess.Popen('sc-controller', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#time.sleep(5)
#sc_proc.kill()

# open the jstest-gtk program to ensure that the controller is connected
#js_proc = subprocess.Popen('jstest-gtk', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#time.sleep(0.5)
#js_proc.kill()

# intialize pygame
pygame.init()

done = False
ready_to_send = True
screen = pygame.display.set_mode((100, 100))
pygame.display.set_caption("My Game")
clock = pygame.time.Clock()

joystick = pygame.joystick.Joystick(0)
joystick.init()

debug_path = "./../../../../../var/www/html/debug.txt"
debug_file = open(r'/var/www/html/debug.txt','wb')

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
    clock.tick(60) #integer value is the frames per second
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

#   print("value of special is: " + str(special))

    if ready_to_send:
        tilt = get_tilt()
        yt = get_yt()
        ready_to_send = True  #needs to be set to False eventually
        data = struct.pack('>Bhhhhh', 37, tilt[0], tilt[1], yt[0], yt[1], special)
        checksum = sum(data) & 0xFF
        data += struct.pack('>B', checksum)
        print("below is sent data:")
        ser.write(data)
        if special == 5:
            # send multiple E-stops to ensure reception
            ser.write(data)
            ser.write(data)
            done = True
            time.sleep(3) # give time to receive all debug UART data before ending
        print(hexify(data))
        timeDiff = time.time() - lastSend
        if (lastSend != -1) and (timeDiff > 0.030):
            loopTime = time.time() - loopTime
            print("ERROR: %f seconds in between packet sends!" % timeDiff)
            print("This loop time: %f. Worst loop time: %f." % (loopTime, worstLoopTime))
            #time.sleep(15)
        lastSend = time.time()
        #print(data)
        #print("ser out waiting " + str(ser.out_waiting))
        print()
        #ser.reset_output_buffer()
        special = 0
    
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

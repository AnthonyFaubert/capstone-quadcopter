import pygame
import time
import serial
import subprocess
import struct

DATA_LENGTH = 10

# open the steam controller driver
sc_proc = subprocess.Popen('sc-controller', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(1)
sc_proc.kill()

# open the jstest-gtk program to ensure that the controller is connected
js_proc = subprocess.Popen('jstest-gtk', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(1)
js_proc.kill()

# intialize pygame
pygame.init()

done = False
ready_to_send = True
screen = pygame.display.set_mode((100, 100))
pygame.display.set_caption("My Game")
clock = pygame.time.Clock()

joystick = pygame.joystick.Joystick(0)
joystick.init()

ser = serial.Serial(port = "/dev/ttyS0", baudrate = 115200, write_timeout = None, timeout = 0) #read timeout should get set to None
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

def send_data(dataValue):
    ser.write(dataValue)
    ser.write(dataValue)
    print(hexify(dataValue))
    print()
    
ser.reset_input_buffer()
ser.reset_output_buffer()
while not done:
    clock.tick(1) #integer value is the frames per second
    
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

#        elif (event.type == pygame.JOYAXISMOTION and (event.axis == 6 or event.axis == 7)):
#            print("axis 6 is " + str(joystick.get_axis(6)))
#            print("axis 7 is " + str(joystick.get_axis(7)))
#            if joystick.get_axis(6) < -0.1:
#                special = 1
#            elif joystick.get_axis(6) > 0.1:
#                special = 2
#            elif joystick.get_axis(7) < -0.1:
#                special = 3
#            elif joystick.get_axis(7) > 0.1:
#                special = 4

#    if (joystick.get_axis(6) > 0.1):
#        special = 1
#    elif (joystick.get_axis(6) < -0.1):
#        special = 2
#    elif (joystick.get_axis(7) > 0.1):
#        special = 3
#    elif (joystick.get_axis(7) < -0.1):
#        special = 4


    print("value of special is: " + str(special))

    if ready_to_send:
        tilt = get_tilt()
        yt = get_yt()
        ready_to_send = True  #needs to be set to False eventually
        data = struct.pack('>hhhhh', tilt[0], tilt[1], yt[0], yt[1], special)
        print("below is sent data:")
        send_data(data)
        print("ser out waiting " + str(ser.out_waiting))
        print()
        time.sleep(1)
        special = 0
    
    if (ser.in_waiting > 0):
        print("below is bytes waiting")
        print(ser.in_waiting)
        ser.ready_to_send = True
        print("****************below is read data:") 
        print(hexify(ser.read(DATA_LENGTH)))
        print()
        ser.reset_input_buffer()

#turn off motors
ser.close()
pygame.quit()

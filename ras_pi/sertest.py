
# coding: utf-8

# In[ ]:

import time 
import serial
import struct 

ser = serial.Serial(port = "/dev/ttyS0", baudrate = 115200, write_timeout = None, timeout = None)


totBytes = 8
delay = 0

x = 1
ser.reset_input_buffer()

while(True):
#    val = struct.pack(b, x)
    val = x.to_bytes(totBytes, byteorder='big')
    ser.write(val)
    #ser.write(val)
    #ser.write(val)
    time.sleep(delay)
    print(str(val) + " " + str(ser.read(totBytes)))
    if (x < 2**(8*totBytes)):
        x = x + 1
    else:
        x = 0

ser.close()


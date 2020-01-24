
# coding: utf-8

# In[ ]:

import time 
import serial
ser = serial.Serial(port = "/dev/ttyS0", baudrate = 9600, write_timeout = 0, timeout = 0)
time.sleep(5)
ser.write(1000)
print(ser.in_waiting)
print("Just slept for 5 seconds.")
print("about to close serial connection")

ser.close()


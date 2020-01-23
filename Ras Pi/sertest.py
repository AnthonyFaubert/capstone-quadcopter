
# coding: utf-8

# In[ ]:


import serial
ser = serial.Serial(port = "/dev/ttyAMA0", baudrate = 9600, write_timeout = 0)
time.sleep(5)
ser.close()


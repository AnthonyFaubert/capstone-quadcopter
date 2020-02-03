import time
import serial
import subprocess
import struct

ser = serial.Serial(port = “/dev/ttyS0”, baudrate = 115200, write_timeout = 0)

a = ‘a’
b = ‘b’
c = ‘c’
d = ‘d’
abcd = ‘abcd’

data1 = struct.pack(‘>cccc’, a, b, c, d)

data2 = struct.pack(’>s’, abcd)

ser.write(data1)
ser.write(data2)
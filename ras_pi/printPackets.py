#!/usr/bin/python3

import struct

DEBUG_FILE = '/var/www/html/debug.txt'

f = open(DEBUG_FILE, 'rb')
packetRaw = f.read(12)
while len(packetRaw) == 12:
    packet = struct.unpack('>BhhhhhB', packetRaw)
    computedChecksum = sum(packetRaw[:-1]) & 0xFF
    actualChecksum = packet[-1]
    if computedChecksum == actualChecksum:
        print('Valid packet: (%d, %d, %d, %d, %d, %d)' % packet[:-1])
    else:
        print('INVALID PACKET: (%d, %d, %d, %d, %d, %d, %d)' % packet)
    packetRaw = f.read(12)

print('Ended with %d bytes left over:' % len(packetRaw), packetRaw)
f.close()

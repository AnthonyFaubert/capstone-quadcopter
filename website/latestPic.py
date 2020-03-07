#!/usr/bin/python3

import glob, os

fileList = glob.glob('/var/www/html/flight_pics/*')
latest = max(fileList, key=os.path.getctime)
latestWebPath = latest[latest.find('flight_pics/'):]

#print('Status: 307 Temporary Redirect')
print('Status: 303 See Other')
print('Location: /%s' % latestWebPath)
print('Content-type:text/plain')
print()

#!/usr/bin/python3

import glob, os

fileList = glob.glob('/var/www/html/flight_logs/*')
latest = max(fileList, key=os.path.getctime)
latestWebPath = latest[latest.find('flight_logs/'):]

#print('Status: 307 Temporary Redirect')
print('Status: 303 See Other')
print('Location: /%s' % latestWebPath)
print('Content-type:text/plain')
print()

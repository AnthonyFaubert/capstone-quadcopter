#!/usr/bin/python3

import glob, os, json

#print("Content-type:text/html\r\n\r\n")
print("Content-type:application/json\r\n\r\n")

fileList = glob.glob('/var/www/html/flight_logs/*')
latest = max(fileList, key=os.path.getctime)

#f = open(latest, 'rb')

# This function written by srohde and edited by Mark Amery on Stackoverflow
# https://stackoverflow.com/questions/2301789/read-a-file-in-reverse-order-using-python
def reverse_readline(filename, buf_size=8192):
    """A generator that returns the lines of a file in reverse order"""
    with open(filename) as fh:
        segment = None
        offset = 0
        fh.seek(0, os.SEEK_END)
        file_size = remaining_size = fh.tell()
        while remaining_size > 0:
            offset = min(file_size, offset + buf_size)
            fh.seek(file_size - offset)
            buffer = fh.read(min(remaining_size, buf_size))
            remaining_size -= buf_size
            lines = buffer.split('\n')
            # The first line of the buffer is probably not a complete line so
            # we'll save it and append it to the last line of the next buffer
            # we read
            if segment is not None:
                # If the previous chunk starts right from the beginning of line
                # do not concat the segment to the last line of new chunk.
                # Instead, yield the segment first 
                if buffer[-1] != '\n':
                    lines[-1] += segment
                else:
                    yield segment
            segment = lines[0]
            for index in range(len(lines) - 1, 0, -1):
                if lines[index]:
                    yield lines[index]
        # Don't yield None if the file was empty
        if segment is not None:
            yield segment

firstSeen = None
data = {}
for line in reverse_readline(latest):
    if (len(line) > 6) and (line[:4] in ['~QC:', '~QJ:', '~QE:', '~P :', '~GC:', '~M :']):
        if line[1:3] == firstSeen:
            break
        elif firstSeen == None:
            firstSeen = line[1:3]
        if line[1] == 'Q':
            quatVals = line.split(':')[-1].strip().split(' ')
            quat = {'w': float(quatVals[0].split('=')[1]), 'x': float(quatVals[1].split('=')[1]), 'y': float(quatVals[2].split('=')[1]), 'z': float(quatVals[3].split('=')[1])}
            quat['s'] = line.split(':')[-1].strip()
            data[line[1:3]] = quat
        elif line[1] == 'P':
            s = line.split(':')[-1].strip().split(' ')
            p = {'r': float(s[0].split('=')[1]), 'p': float(s[1].split('=')[1]), 'y': float(s[2].split('=')[1])}
            p['s'] = line.split(':')[-1].strip()
            data['P'] = p
        else:
            data[line[1:3]] = {'s': line.split(':')[-1].strip()}

print(json.dumps(data))

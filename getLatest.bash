#!/bin/bash

if [ $1 ]; then
    curl -L http://10.144.10.10/latestDataDump.py > $1
    echo "Wrote $1"
else
    echo "Usage: getLatest.bash [outputfile]"
fi

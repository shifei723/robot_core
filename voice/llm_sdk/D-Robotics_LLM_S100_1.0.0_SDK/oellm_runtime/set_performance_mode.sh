#!/bin/sh

boardid=$(hrut_boardid | awk '{print substr($0,1,4)}')

if [ "$boardid" = "0x64" ]; then
    echo "Set S100P performance mode"
    devmem 0x2b047000 32 0x99
    devmem 0x2b047004 32 0x99
else
    echo "This board is not S100P, exit"
fi

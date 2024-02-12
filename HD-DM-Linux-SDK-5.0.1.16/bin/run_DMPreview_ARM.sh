#!/bin/sh
echo "run ARM"
export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
export LD_LIBRARY_PATH=../eSPDI/opencv/ARMHF_32/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/turbojpeg/armhf_32/lib/:$LD_LIBRARY_PATH 
sync
./DMPreview_ARM

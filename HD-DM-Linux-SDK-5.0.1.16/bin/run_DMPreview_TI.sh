#!/bin/sh
echo "run TI"
export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
export LD_LIBRARY_PATH=../eSPDI/opencv/armhf_ti_32/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../eSPDI/turbojpeg/armhf_ti_32/lib/:$LD_LIBRARY_PATH 
sync
./DMPreview_TI

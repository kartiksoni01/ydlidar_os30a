#!/bin/sh
SUCCESS=0
export LD_LIBRARY_PATH=./../eSPDI:$LD_LIBRARY_PATH 
export LD_LIBRARY_PATH=./../eSPDI/turbojpeg/armhf_32/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./../eSPDI/opencv/ARMHF_32/lib/:$LD_LIBRARY_PATH

cd out_img
if [ "$?" -ne $SUCCESS ]
then
	echo "creat out img folder"
	mkdir out_img
else
    cd ../
    rm -rf ./out_img/*.*
	echo "run ARM test..."
fi
./test_armhf

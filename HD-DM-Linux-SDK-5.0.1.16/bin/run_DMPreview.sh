#!/bin/sh

echo "run DMPreview tools, please select CPU type: "
echo "=================="
echo "1. X86-64"
echo "2. ARM"
echo "3. TI"
echo "4. NVIDIA TX2"
echo "=================="
echo 

read -p "Please select CPU type (enter: 1(x86_64), 2(armhf_32), 3(TI), 4(NVIDIA TX2/NVIDIA Nano)) : " project

case $project in
        [1]* ) 
        echo "run x86_64"
        ./DMPreview_X86
        break;;

        [2]* ) 
        echo "run ARM"
		export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
        export LD_LIBRARY_PATH=../eSPDI/opencv/ARMHF_32/lib/:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=../eSPDI/turbojpeg/armhf_32/lib/:$LD_LIBRARY_PATH 
        sync
        ./DMPreview_ARM
		break;;

        [3]* ) 
        echo "run TI"
		export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
        export LD_LIBRARY_PATH=../eSPDI/opencv/armhf_ti_32/lib/:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=../eSPDI/turbojpeg/armhf_ti_32/lib/:$LD_LIBRARY_PATH 
        sync
        ./DMPreview_TI
		break;;
        
        [4]* ) 
        echo "run NVIDIA TX2"
		export LD_LIBRARY_PATH=../eSPDI:$LD_LIBRARY_PATH 
        export LD_LIBRARY_PATH=../eSPDI/opencv/TX2/lib/:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=../eSPDI/turbojpeg/TX2/lib/:$LD_LIBRARY_PATH 
        sync
        ./DMPreview_TX2
		break;;
        * ) 
        echo "Please select CPU type";;
esac

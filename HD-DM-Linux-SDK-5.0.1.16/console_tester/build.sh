#!/bin/sh

echo "Project list : "
echo "=================="
echo "1. x86-64"
echo "2. NVIDIA TX2"
echo "3. Rockchip PX30"
echo "4. MTK"
echo "5. TI (ARMHF)"
echo "=================="
read -p "Please select project: " project

case $project in
        [1]* ) 
        echo "build x86_64 console tester"
                make CPU=X86 BITS=64 clean
                make CPU=X86 BITS=64
                break;;
        [2]* ) 
        echo "build NVIDIA TX2 console tester"
                make CPU=NVIDIA BITS=64 clean
                make CPU=NVIDIA BITS=64
		break;;
		[3]* ) 
        echo "build Rockchip PX30 console tester"
                make CPU=PX30 BITS=64 clean
                make CPU=PX30 BITS=64
		break;;
        [4]* ) 
        echo "build MTK console tester"
                make CPU=MTK BITS=64 clean
                make CPU=MTK BITS=64
		break;;
        [5]* ) 
        echo "build TI (ARMHF) console tester"
                make CPU=TI BITS=32 clean
                make CPU=TI BITS=32
		break;;
        * ) 
        echo "Please selet project no.";;
esac

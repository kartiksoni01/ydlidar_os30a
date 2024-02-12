#!/bin/bash

export LD_LIBRARY_PATH=./../eSPDI:$LD_LIBRARY_PATH
echo -n "iteration count："
read A
B=0

while [ $B -lt $A ]
do
    echo ">>>>>>>>>>>>>>>> iteration #"$B
    rm -fr ./out_img
    mkdir ./out_img
    cat 0.cmd | ./test_arm64_mtk
    B=$(($B+1))
done



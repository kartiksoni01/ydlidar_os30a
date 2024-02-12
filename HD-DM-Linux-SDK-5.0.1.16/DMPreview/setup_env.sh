#!/bin/sh

echo "1. Install default environment package (Ubuntu x86_64)"
echo "======================================================"
sudo apt-get update
sudo apt-get upgrade
sudo apt update
sudo apt upgrade
sudo apt-get install build-essential qtcreator qt5-default libudev-dev cmake libxt-dev libusb-1.0-0-dev libwebp-dev
sudo apt-get install meshlab -y
sudo apt-get install doxygen
sudo apt-get install libdc1394-22 libdc1394-22-dev
sudo apt install ocl-icd-opencl-dev
#sudo apt install zlib1g
#sudo apt install zlib1g-dev
#sudo apt-get install libzip-dev
#sudo apt-get install libssl-dev


echo
echo
echo "2. Install Grphics card driver (depend graphics card vendor)"
echo "============================================================"
echo "List can install drivers:"
echo "!!! Please install recommended driver (choose [recommended] driver) as below command !!!"
echo "for example: sudo apt-get install nvidia-driver-450-server"
echo "!!! Must reboot after installation !!!"
echo
sudo ubuntu-drivers devices
echo

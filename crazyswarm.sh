#!/bin/bash
# For further informations
# please refers to https://imrclab.github.io/crazyswarm2/installation.html 

# Install dependencies
sudo apt install libboost-program-options-dev libusb-1.0-0-dev
pip3 install rowan nicegui

sudo apt-get install ros-humble-motion-capture-tracking

pip3 install cflib transforms3d
sudo apt-get install ros-humble-tf-transformations

# Set up Crazyradio
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER

# Set up software-in-the-loop simulation
cd
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
export PYTHONPATH=~/crazyflie-firmware/build:$PYTHONPATH
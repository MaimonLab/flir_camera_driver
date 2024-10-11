#!/bin/bash

# prepare installer tools 
sudo apt-get update
sudo apt install -y python3-pip
sudo apt-get install -y wget

# install python packages required for spinnaker
cd ~/maimon_ws/src
git clone git@github.com:MaimonLab/maimon_classes.git
pip3 install opencv-python
# pip3 install opencv-contrib-python

# navigate to pre-extracted Spinnaker SDK files on server
cd /mnt/maimondata01/lab_resources/software/

# install other dependencies 
sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0

# run the main installer script 
./spinnaker-3.2.0.62-amd64-pkg.20.04/spinnaker-3.2.0.62-amd64/install_spinnaker.sh

# repeat for python .whl file
cd /mnt/maimondata01/lab_resources/software/spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64
sudo python3.8 -m pip install --upgrade numpy matplotlib
pip3 install spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64.whl
pip3 install ruamel.yaml

# update firmware for Chameleon3 cameras
SpinUpdateConsole -R.* -B /mnt/maimondata01/lab_resources/software/cmln3-python1300_v1.2v1.3_cm3-u3-13y3-1.24.2-01.ez2

echo "----------------------------------------------"
echo "--------Finished installing flir_camera_driver"
echo "----------------------------------------------"
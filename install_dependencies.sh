#!/bin/bash

cwd=$(pwd)

# prepare installer tools 
sudo apt-get update
sudo apt install -y python3-pip
sudo apt-get install -y wget

# install python packages required for spinnaker
cd ~/maimon_ws/src
git clone git@github.com:MaimonLab/maimon_classes.git
pip3 install opencv-python

# This is the default mounted location for the data server,
# but can be specified otherwise with the -get_from flag
get_from_loc="/mnt/maimondata01/lab_resources/software/"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -get_from) get_from_loc="$2"; shift ;;
        *) echo "Using default file location: get_from_loc";;
    esac
    shift
done

# navigate to pre-extracted Spinnaker SDK files on server
cd $get_from_loc

# install other dependencies
sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0

# run the main installer script
sudo bash ./spinnaker-3.2.0.62-amd64-pkg.20.04/spinnaker-3.2.0.62-amd64/install_spinnaker.sh

# repeat for python .whl file
cd $get_from_loc/spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64
sudo python3.8 -m pip install numpy matplotlib
pip3 install spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64.whl
pip3 install ruamel.yaml

## update firmware for Chameleon3 cameras
devices=$( lsusb -d 0x1e10:0x3300 -v | grep iSerial | awk '{ print $3 }' )
for dev in $devices; do
  SpinUpdateConsole -R$((16#$dev)) -B $get_from_loc/cmln3-python1300_v1.2v1.3_cm3-u3-13y3-1.24.2-01.ez2
done

echo "----------------------------------------------"
echo "--------Finished installing flir_camera_driver"
echo "----------------------------------------------"

cd $cwd

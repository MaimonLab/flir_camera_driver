#!/bin/bash

cwd=$(pwd)

# prepare installer tools
sudo apt-get update
sudo apt install -y python3-pip
sudo apt-get install -y wget

# install python packages required for spinnaker
if ! [[ -z "${CURRENT_WS}" ]]; then
    cd "${CURRENT_WS}/src"
else
    cd /home/maimon/maimon_ws/src
fi
echo "Cloning required repositories to: $(pwd)"
git clone git@github.com:MaimonLab/maimon_classes.git

# install other dependencies
sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0


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
echo "Fetching Spinnaker SDK files from: ${get_from_loc}"

# check relevant versions
DISTRIB_RELEASE=$(cat /etc/*-release | grep DISTRIB_RELEASE)
DISTRIB_RELEASE_arr=(${DISTRIB_RELEASE//=/ })
DISTRIB_RELEASE_V=${DISTRIB_RELEASE_arr[1]}
PYTHON_VERSION=($(python3 --version))
echo "Installing Spinnaker build for: Ubuntu ${DISTRIB_RELEASE_V}, Python ${PYTHON_VERSION[1]}"

# run the main installer script
if [[ $DISTRIB_RELEASE_V=="20.04" ]]; then
    cd ./spinnaker-3.2.0.62-amd64-pkg.20.04/spinnaker-3.2.0.62-amd64/
elif [[ $DISTRIB_RELEASE_V=="22.04" ]]; then
    cd ./spinnaker-3.2.0.62-amd64.22.04/
fi

if ! (sudo test -x ./install_spinnaker.sh); then
  echo "------------------------------------------------------------"
  echo "---ERROR: Permission Denied for Spinnaker install script!---"
  echo "------------------------------------------------------------"
  exit 1
fi

sudo bash ./install_spinnaker.sh

cd $get_from_loc

# repeat for python .whl file
if [[ ${PYTHON_VERSION[1]}=="3.8"* ]]; then
    cd $get_from_loc/spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64
    sudo python3.8 -m pip install numpy==1.* matplotlib opencv-python
    pip3 install spinnaker_python-3.2.0.62-cp38-cp38-linux_x86_64.whl
elif [[ ${PYTHON_VERSION[1]}=="3.10"* ]]; then
    sudo python3.10 -m pip install numpy==1.* matplotlib opencv-python
    pip3 install $get_from_loc/spinnaker_python-3.2.0.62-cp310-cp310-linux_x86_64.whl
fi
pip3 install ruamel.yaml

echo "---------------------------------------------------"
echo "---Finished installing Spinnaker SDK!--------------"
echo "---If the following camera firmware update fails---"
echo "---RESTART your computer!--------------------------"
echo "---------------------------------------------------"

# update firmware for Chameleon3 cameras
devices=$( lsusb -d 0x1e10:0x3300 -v | grep iSerial | awk '{ print $3 }' )
for dev in $devices; do
  SpinUpdateConsole -R$((16#$dev)) -B $get_from_loc/cmln3-python1300_v1.2v1.3_cm3-u3-13y3-1.24.2-01.ez2
done

echo "----------------------------------------------"
echo "---Finished installing flir_camera_driver-----"
echo "----------------------------------------------"

cd $cwd

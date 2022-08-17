#!/bin/bash

# prepare installer tools 
sudo apt-get update
sudo apt install -y python3-pip
sudo apt-get install -y wget

# install python packages required for spinnaer
pip3 install opencv-python
pip3 install opencv-contrib-python


# download spinner to the Downloads folder ,create if this doens't exist yet
mkdir -p ~/Downloads
cd ~/Downloads

# Donwload and unzip 
wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABnzTO7hqbQ5v7ndBbzEevwa/spinnaker/spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz
tar -xzf   spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz
cd spinnaker-2.3.0.77-amd64

# install other dependencies 
sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0



# install spinnaer specific pacakges
echo "Installing Spinnaker packages..."

# this asks you to accept the agreement , this is the only yes you need to click 
yes | sudo dpkg -i libspinnaker_*.deb

sudo dpkg -i libspinnaker-dev_*.deb
sudo dpkg -i libspinnaker-c_*.deb
sudo dpkg -i libspinnaker-c-dev_*.deb
sudo dpkg -i libspinvideo_*.deb
sudo dpkg -i libspinvideo-dev_*.deb
sudo dpkg -i libspinvideo-c_*.deb
sudo dpkg -i libspinvideo-c-dev_*.deb
sudo dpkg -i spinview-qt_*.deb
sudo dpkg -i spinview-qt-dev_*.deb
sudo dpkg -i spinupdate_*.deb
sudo dpkg -i spinupdate-dev_*.deb
sudo dpkg -i spinnaker_*.deb
sudo dpkg -i spinnaker-doc_*.deb
sudo dpkg -i libgentl_*.deb


# add current user to user group
grpname="flirimaging"

echo "Adding new members to usergroup $grpname..." 
usrname = $USER
groupadd -f $grpname
usermod -a -G $grpname $usrname
echo "Added user $usrname to $grpname"

# Create udev rule
UdevFile="/etc/udev/rules.d/40-flir-spinnaker.rules"
echo "Writing the udev rules file...";
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1e10\", GROUP=\"$grpname\"" 1>>$UdevFile

echo "Restarting udev daemon"
/etc/init.d/udev restart

echo "UDEV Configuration complete."
echo "A reboot may be required on some systems for changes to take effect."

# usb to 1000 MB/s 
sudo sh configure_usbfs.sh

# some path stuff? 
sudo sh configure_spinnaker_paths.sh


# Download spinnaker_python 
cd ~/Downloads
wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABu07_NN5hMkwb-W_E69wFja/spinnaker/spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz

# create a directory and unzip it in there (this is a tarball) 
mkdir spinnaker_python-2.3.0.77
tar -xzf  spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz -C spinnaker_python-2.3.0.77
cd spinnaker_python-2.3.0.77

# install spinnaaker_python dependencies 
sudo python3.8 -m pip install --upgrade numpy matplotlib

# install spinnaker_python wheel 
pip3 install spinnaker_python-2.3.0.77-cp38-cp38-linux_x86_64.whl 

# install simple pyspin, an easy wrapper around pyspin
pip3 install simple-pyspin

echo "------------------------------------------"
echo "--------Installation complete.------------"
echo "------------------------------------------"
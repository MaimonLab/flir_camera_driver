
sudo apt-get update
sudo apt install -y python3-pip

pip3 install opencv-python
pip3 install opencv-contrib-python

mkdir -p ~/src/spinnaker
cd ~/src/spinnaker

wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABnzTO7hqbQ5v7ndBbzEevwa/spinnaker/spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz

tar -xvzf   spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz
cd spinnaker-2.3.0.77-amd64

sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0

echo "Installing Spinnaker packages..."

sudo dpkg -i libspinnaker_*.deb
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


grpname="flirimaging"

echo "Adding new members to usergroup $grpname..." 
usrname = $USER
groupadd -f $grpname
usermod -a -G $grpname $usrname
echo "Added user $usrname"

# Create udev rule
UdevFile="/etc/udev/rules.d/40-flir-spinnaker.rules"
echo "Writing the udev rules file...";
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1e10\", GROUP=\"$grpname\"" 1>>$UdevFile

echo "Restarting udev daemon"
confirm = "y"
if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
then
    /etc/init.d/udev restart
else
    echo "Udev was not restarted.  Please reboot the computer for the rules to take effect."
    exit 0
fi

echo "UDEV Configuration complete."
echo "A reboot may be required on some systems for changes to take effect."
exit 0

sudo sh configure_usbfs.sh

sudo sh configure_spinnaker_paths.sh

cd ~/src/spinnaker
wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABu07_NN5hMkwb-W_E69wFja/spinnaker/spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz

tar -xvzf  spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz
cd spinnaker_python-2.3.0.77

sudo python3.8 -m pip install --upgrade numpy matplotlib
pip3 install spinnaker_python-2.3.0.77-cp38-cp38-linux_x86_64.whl 
pip3 install simple-pyspin

echo
echo "Installation complete."

sudo apt-get update
sudo apt install -y python3-pip

pip3 install opencv-python
pip3 install opencv-contrib-python

mkdir -p ~/src/spinnaker
cd ~/src/spinnaker
# WKJ
# wget https://www.dropbox.com/s/m2vwc5j1o58tt7f/spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz?dl=0
wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABnzTO7hqbQ5v7ndBbzEevwa/spinnaker/spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz?dl=0


tar -xvzf   spinnaker-2.3.0.77-Ubuntu20.04-amd64-pkg.tar.gz
# cd spinnaker-2.2.0.48-amd64
cd spinnaker-2.3.0.77-amd64

# unzip 

sudo apt-get install -y libavcodec58 libavformat58 \
libswscale5 libswresample3 libavutil56 libusb-1.0-0 \
libpcre2-16-0 libdouble-conversion3 libxcb-xinput0 \
libxcb-xinerama0

cd spinnaker-2.3.0.77-amd64
# sudo sh install_spinnaker.sh

# #!/bin/bash

# set -o errexit

# MY_YESNO_PROMPT='[Y/n] $ '

# Web page links
# FEEDBACK_PAGE='https://www.flir.com/spinnaker/survey'

# echo "This is a script to assist with installation of the Spinnaker SDK."
# echo "Would you like to continue and install all the Spinnaker SDK packages?"
# echo -n "$MY_YESNO_PROMPT"
# read confirm
# if ! ( [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ] )
# then
#     exit 0
# fi

# echo

# set +o errexit
# EXISTING_VERSION=$(dpkg -s spinnaker 2> /dev/null | grep '^Version:' | sed 's/^.*: //')
# set -o errexit

# if [ ! -z "$EXISTING_VERSION" ]; then
#     echo "A previous installation of Spinnaker has been detected on this machine (Version: $EXISTING_VERSION). Please consider uninstalling the previous version of Spinnaker before continuing with this installation." >&2
#     echo "Would you like to continue with this installation?"
#     echo -n "$MY_YESNO_PROMPT"
#     read confirm
#     if ! ( [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ] )
#     then
#         exit 0
#     fi
# fi

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

# echo
# echo "Would you like to add a udev entry to allow access to USB hardware?"
# echo "  If a udev entry is not added, your cameras may only be accessible by running Spinnaker as sudo."
# echo -n "$MY_YESNO_PROMPT"
# read confirm
# if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
# then
#     echo "Launching udev configuration script..."
#     sudo sh configure_spinnaker.sh
# fi
# sudo sh configure_spinnaker.sh
set -o errexit

MY_PROMPT='$ '
MY_YESNO_PROMPT='[Y/n] $ '

grpname="flirimaging"

if [ "$(id -u)" = "0" ]
then
    echo
    echo "This script will assist users in configuring their udev rules to allow"
    echo "access to USB devices. The script will create a udev rule which will"
    echo "add FLIR USB devices to a group called $grpname. The user may also"
    echo "choose to restart the udev daemon. All of this can be done manually as well."
    echo
else
    echo
    echo "This script needs to be run as root, e.g.:"
    echo "sudo configure_spinnaker.sh"
    echo
    exit 0
fi

echo "Adding new members to usergroup $grpname..." 
while :
do
    # Show current members of the user group
    users=$(grep -E '^'$grpname':' /etc/group |sed -e 's/^.*://' |sed -e 's/, */, /g')
    if [ -z "$users" ]
    then 
        echo "Usergroup $grpname is empty"
    else
        echo "Current members of $grpname group: $users"
    fi

    echo "To add a new member please enter username (or hit Enter to continue):"
    # echo -n "$MY_PROMPT"
    # read usrname
    usrname = "maimon"
    if [ "$usrname" = "" ]
    then
        break
    else
        # Check if user name exists
        if (getent passwd $usrname > /dev/null)
        then
            # Get confirmation that the username is ok 
            echo "Adding user $usrname to group $grpname group. Is this OK?"
            echo -n "$MY_YESNO_PROMPT"
            read confirm
            if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
            then
                # Create user group (if not exists) and add user to it
                groupadd -f $grpname
                usermod -a -G $grpname $usrname
                echo "Added user $usrname"
            fi
        else
            echo "User "\""$usrname"\"" does not exist"
        fi
    fi
done

# Create udev rule
UdevFile="/etc/udev/rules.d/40-flir-spinnaker.rules"
echo
echo "Writing the udev rules file...";
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1e10\", GROUP=\"$grpname\"" 1>>$UdevFile

echo "Do you want to restart the udev daemon?"
# echo -n "$MY_YESNO_PROMPT"
# read confirm
confirm = "y"
if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
then
    /etc/init.d/udev restart
else
    echo "Udev was not restarted.  Please reboot the computer for the rules to take effect."
    exit 0
fi

echo "Configuration complete."
echo "A reboot may be required on some systems for changes to take effect."
exit 0


# echo
# echo "Would you like to set USB-FS memory size to 1000 MB at startup (via /etc/rc.local)?"
# echo "  By default, Linux systems only allocate 16 MB of USB-FS buffer memory for all USB devices."
# echo "  This may result in image acquisition issues from high-resolution cameras or multiple-camera set ups."
# echo "  NOTE: You can set this at any time by following the USB notes in the included README."
# echo -n "$MY_YESNO_PROMPT"
# read confirm
# if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
# then
#     echo "Launching USB-FS configuration script..."
#     sudo sh configure_usbfs.sh
# fi
sudo sh configure_usbfs.sh

# echo

# echo "Would you like to have Spinnaker prebuilt examples available in your system path?"
# echo "  This allows Spinnaker prebuilt examples to run from any paths on the system."
# echo "  NOTE: You can add the Spinnaker example paths at any time by following the \"RUNNING PREBUILT UTILITIES\""
# echo "        section in the included README."
# echo -n "$MY_YESNO_PROMPT"

# read confirm
# if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
# then
#     echo "Launching Spinnaker paths configuration script..."
#     sudo sh configure_spinnaker_paths.sh
# fi
sudo sh configure_spinnaker_paths.sh

echo

ARCH=$(ls libspinnaker_* | grep -oP '[0-9]_\K.*(?=.deb)' || [[ $? == 1 ]])
if [ "$ARCH" = "amd64" ]; then
    BITS=64
elif [ "$ARCH" = "i386" ]; then
    BITS=32
fi

# if [ -z "$BITS" ]; then
#     echo "Could not automatically add the FLIR GenTL Producer to the GenTL environment variable."
#     echo "To use the FLIR GenTL Producer, please follow the GenTL Setup notes in the included README."
# else
#     echo "Would you like to have the FLIR GenTL Producer added to GENICAM_GENTL${BITS}_PATH?"
#     echo "  This allows GenTL consumer applications to load the FLIR GenTL Producer."
#     echo "  NOTE: You can add the FLIR producer to GENICAM_GENTL${BITS}_PATH at any time by following the GenTL Setup notes in the included README."
#     echo -n "$MY_YESNO_PROMPT"

#     read confirm
#     if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
#     then
#         echo "Launching GenTL path configuration script..."
#         sudo sh configure_gentl_paths.sh $BITS
#     fi
# fi

echo
echo "Installation complete."

# echo
# echo "Would you like to make a difference by participating in the Spinnaker feedback program?"
# echo -n "$MY_YESNO_PROMPT"
# read confirm
# if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ]
# then
#     feedback_link_msg="Please visit \"$FEEDBACK_PAGE\" to join our feedback program!"
#     if [ $(id -u) -ne 0 ]
#     then
#         set +o errexit
#         has_display=$(xdg-open $FEEDBACK_PAGE 2> /dev/null && echo "ok")
#         set -o errexit
#         if [ "$has_display" != "ok" ]
#         then
#             echo $feedback_link_msg
#         fi
#     elif [ "$PPID" -ne 0 ]
#     then
#         # Script is run as sudo. Find the actual user name.
#         gpid=$(ps --no-heading -o ppid -p $PPID)
#         if [ "$gpid" -ne 0 ]
#         then
#             supid=$(ps --no-heading -o ppid -p $gpid)
#             if [ "$supid" -ne 0 ]
#             then
#                 user=$(ps --no-heading -o user -p $supid)
#             fi
#         fi

#         if [ -z "$user" ] || [ "$user" = "root" ]
#         then
#             # Root user does not have graphical capabilities.
#             echo $feedback_link_msg
#         else
#             set +o errexit
#             has_display=$(su $user xdg-open $FEEDBACK_PAGE 2> /dev/null && echo "ok")
#             set -o errexit
#             if [ "$has_display" != "ok" ]
#             then
#                 echo $feedback_link_msg
#             fi
#         fi
#     else
#         echo $feedback_link_msg
#     fi
# else
#     echo "Join the feedback program anytime at \"$FEEDBACK_PAGE\"!"
# fi

# echo "Thank you for installing the Spinnaker SDK."
# exit 0





cd ~/src/spinnaker
# wget https://www.dropbox.com/s/jrgzcv149lfb354/spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz?dl=0
wget https://www.dropbox.com/sh/xod2fj9wqq0ie82/AABu07_NN5hMkwb-W_E69wFja/spinnaker/spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz?dl=0

tar -xvzf  spinnaker_python-2.3.0.77-Ubuntu20.04-cp38-cp38-linux_x86_64.tar.gz
cd spinnaker_python-2.3.0.77

sudo python3.8 -m pip install --upgrade numpy matplotlib
pip3 install spinnaker_python-2.3.0.77-cp38-cp38-linux_x86_64.whl 
pip3 install simple-pyspin

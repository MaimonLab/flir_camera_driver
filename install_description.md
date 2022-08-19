# Install Description

Install python dependencies

    pip3 install opencv-python
    pip3 install opencv-contrib-python

We'll need to install **Spinnaker**, **spinnaker_python** and **pyspin-simple**.

1.  **Download and install Spinnaker** You can download [spinnaker](https://www.flir.com/products/spinnaker-sdk/) from FLIR's website. At the website go to:

    - Download Now
    - submit
    - Spinnaker
    - Linux Ubuntu
    - Ubuntu 20.04
    - Download the one with amd64 (usually the top one)

    **Note: the FLIR download page is occasionally not functional. Alternatively to downloading, you can get the files from the maimonlab NAS -> FLIR_spinnaker_downloads.**

    Unzip spinnaker:

         cd ~/Downloads
         tar -xvzf spinnaker-2.2.0.48-Ubuntu20.04-amd64-pkg.tar.gz
         cd spinnaker-2.2.0.48-amd64

    (press tab twice to tab-complete the full spinnaker filename). Then open the readme

         xdg-open README

    Scroll to dependency installation and copy the multiline starting with `sudo apt-get install ...`. that into the terminal. (CTRL-SHIFT-V to paste). Then start installation with:

         sudo sh install_spinnaker.sh

    During the installation you will be asked about the following settings:

    - Do you want to add a user to the **udev permission**? Here you want to say [yes], and add the username (`maimon` in my case). If you do not add this setting, your PointGrey USB cameras will not be detected in spinview and the spinnaker SDK.
    - You need an increased [usbfs](https://www.flir.com/support-center/iis/machine-vision/application-note/using-linux-with-usb-3.1/) memory. Ubuntu has it set to 16 Mb/s by default, but FLIR recommends setting this to 1000 Mb.

    Your camera won't show up until you restart because the permission takes effect only after a login.

2.  **Spinnaker-pyspin**
    Spinnaker pyspin is the python interface with spinnaker. You can download [spinnaker-python](https://www.flir.com/products/spinnaker-sdk/) from FLIR's website. At the website go to:

    - Download
    - submit
    - Spinnaker
    - Linux Ubuntu
    - Python
    - Ubuntu 20.04
    - x64
    - Download the file
      The zip file comes with a readme on how to install it.

    Unzip the file into a folder:

        cd ~/Downloads
        mkdir spinnaker_python
        tar -xvzf spinnaker_python-<tab-tab-to-complete>.tar.gz -C spinnaker_python
        cd spinnaker_python

    Open the readme:

        xdg-open README.txt

    The readme will have installation instructions, but basically you'll need to install pip, and use pip to install the spinnaker_python-xxx.whl file.

        sudo apt install python3-pip
        sudo python3.8 -m pip install --upgrade numpy matplotlib
        pip3 install spinnaker_python-2.<tab-tab-to-complete>.whl

    To simplify the interfacing with the pyspin library, the `flir_camera_driver` use the [simple_pyspin](https://github.com/klecknerlab/simple_pyspin) wrapper. We'll install it with:

        pip3 install simple-pyspin

# flir_camera_driver

This ROS2 package publishes images from FLIR cameras to an image topic. You can set general node settings (e.g. image_topic) and specific camera settings (e.g. AcquisitionFrameRate) by supplying a config file during launch of the node.

Content:

- [Example](#example)
- [Installation](#installation)
- [Gotchas](#gotchas)
- [Release Notes](#release)

<a name=example></a>

# Example

The basic example opens a camera publisher and an rqt_image_viewer:

    ros2 launch flir_camera_driver example.launch.py

In this exmaple the parameters from `/config/example_config.yaml` under the header `basic_example_config` are passed to the camera node.

<a name=installation></a>

# Installation

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

# Gotchas

<a name=gotchas></a>

- **PARAMETER ORDER OF APPLICATION** If you have worked with FLIR's Spinview before you might have noticed that some setting options are blocked until another setting is in the right state. An example is that the "Acqusition Frame Rate Auto" needs to be set to "Off" before you can set "Acqusition Frame Rate" to a specific value. Spinnaker also has this issue, and when we run the `publish_pyspin_simple` node and we have a dictionary of settings we want to apply to the camera, we cannot guarrantee that the dictionary entries are applied in the right order to prevent settings to be blocked. This happens frequently, and by default a blocked setting will be skipped while logging a warning.

  **Hacky solution:** To circumvent this unknown settings order the publish_pyspin_simple uses a list of settings named `CAMERA_PRIORITY_SET_ORDER`. The node will first go through this list and set parameters according to this order, before setting parameters not in this list. The `CAMERA_PRIORITY_SET_ORDER` is by no means exhaustive, it merely reflects parameters frequently used in the maimon lab.

- **YAML TYPE CONVERSION** Yaml has the unfortunate property of interpreting Off without quotes as False. Since we need to set the parameter as the string "Off", this will lead to an error if unhandled.
  **Hacky Solution:** The list `PARAMETER_FALSE_MEANS_OFF` contains parameters that if it detects a false, this will be turned into the string "Off". This list is by no means exhaustive and if you have another parameter with this property you must add it to the list.

**What parameters are available?** The simple_pyspin package comes with [example scripts](https://klecknerlab.github.io/simple_pyspin/) and a [list of settings for the Chameleon cameras](https://klecknerlab.github.io/simple_pyspin/cameras/Point_Grey_Research_Chameleon3_CM3-U3-13Y3M.html).

# License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.

<a name=release></a>

# Release Notes

**0.1.x** Update readme to include installation instructions
**0.1.0** Tested on Chameleon CM3-U3-13Y3M

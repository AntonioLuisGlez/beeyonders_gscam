# gscam for RTP connection

This submodule is a modified version of [gscam](https://github.com/ros-drivers/gscam).
The original version has two main issues: a problem with the Gstreamer version (More info: [#61](https://github.com/ros-drivers/gscam/pull/61)), and an issue when using an RTP server as a source in the pipeline provided through `gscam_config`. To address these issues, the following changes have been made compared to the original repository:

1. The change proposed in the following pull request has been applied: [#61](https://github.com/ros-drivers/gscam/pull/61)
2. In [gscam.cpp](src/gscam_RTP_pkg/src/src/gscam.cpp), the launchpipe element that contained the pipeline information described in `gscam_config` has been replaced with the combination of the created and configured pipeline elements within the same code. In this version of gscam **there is no need to pass any Gstreamer pipeline either from an environment variable or ROS params.**.
3. The variables and lines of code handling the `gscam_config` variable have been removed.
4. The ROS parameter `UDP_port` has been added to select which UDP port should be used to receive the images.
5. The ROS parameter `use_h265` has been added to select between h264 decoder or h265 decoder.
6. [RTP_to_ros.launch](src/gscam_RTP_pkg/src/examples/RTP_to_ros.launch) has been added to launch the node for a RTP connection.
7. [RTP_to_ros_two_cameras.launch](src/gscam_RTP_pkg/src/examples/RTP_to_ros_two_cameras.launch) has been added to launch the node for two RTP connection.
7. [RTP_qualcomm.launch](src/gscam_RTP_pkg/src/examples/RTP_to_ros.launch) has been added to launch the node for RTP qualcomm with HADRON 640R connection. This file used the 'remote_cmd_executor' package.

## Usage

### Generic RTP streaming

The following instructions are used to connect to an RTP stream and convert it to ROS messages. 

1.  **Build gscam_RTP_pkg**: Build gscam with the following command:
    ```
    catkin_make -DGSTREAMER_VERSION_1_x=On -j4
    ```

2.  **Launch the ros node for the RTP communication**: After building, you can run the ROS node for an RTP stream with:
    ```
    source devel/setup.zsh
    roslaunch gscam RTP_to_ros.launch UDP_PORT:={Source UDP port} USE_H265:={true for H265, false for H264}
    ```
    Here, "Source UDP port" is the UDP port you want to connect to, and "true for H265, false for H264" specifies the encoder type: true for H265 and false for H264.

    In case you want to run the ROS node for two simultaneous RTP streams:
    ```
    source devel/setup.zsh
    roslaunch gscam RTP_to_ros.launch UDP_PORT:={Source UDP port} USE_H265:={true for H265, false for H264}
    ```
    Here, "Source UDP port" is the UDP port you want to connect to, and "true for H265, false for H264" specifies the encoder type: true for H265 and false for H264.

### Qualcomm with FLIR HADRON 640R integration RTP streaming

The following instructions are to make the Qualcomm with FLIR HADRON 640R create the RTP video stream and, on the computer side, take the video stream and convert it to ROS messages.

1. **Configure a SSH access**:
    - Create a SSH key in yout computer: The next command create two SSH keys (One is public and the other one is private)
        ```
        ssh-keygen -t rsa
        ```
        During this process, you will be prompted to choose a location to save the key (default is ~/.ssh/id_rsa) and to enter a passphrase (optional). You can press Enter to accept the default options and omit the passphrase.

        The output would look like this:
        ```
        Generating public/private rsa key pair.
        Enter file in which to save the key (/home/yourusername/.ssh/id_rsa): [Press Enter]
        Enter passphrase (empty for no passphrase): [Press Enter]
        Enter same passphrase again: [Press Enter]
        ```
    - Copy public key to Qualcomm: This command copies the public key generated on your computer to the ~/.ssh/authorized_keys file on the Qualcomm, allowing passwordless authentication.
        ```
        ssh-copy-id username@qualcomm_ip
        ```
        Replace username with your Qualcomm username and qualcomm_ip with the Qualcomm IP address. You will be prompted to enter your Qualcomm account password once to perform the copy. If you want to see what is your Qualcomm username you can type `who` in the qualcomm terminal, and if you want to see the Qualcomm IP address you can type `ip addr show` in the computer terminal.

        The output would look like this:
        ```
        The authenticity of host 'qualcomm_ip (qualcomm_ip)' can't be established.
        RSA key fingerprint is SHA256:...
        Are you sure you want to continue connecting (yes/no)? yes
        username@qualcomm_ip's password: [Enter your password]
        ```
    - Verify SSH connection without password: You should now be able to connect to Qualcomm from your computer without being prompted for a password.
        ```
        ssh username@qualcomm_ip
        ```

2. **Build gscam_RTP_pkg and remote_cmd_executor packages**:
    You can build gscam_RTP_pkg and remote_cmd_executor packages with the following command:
    ```
    catkin build -DGSTREAMER_VERSION_1_x=On -j4
    ```
    Or using catkin_make:
    ```
    catkin_make -DGSTREAMER_VERSION_1_x=On -j4
    ```
3. **Launch the ros node for the RTP qualcomm communication**:
    
    Change the `username` and `qualcomm_ip` parameters in [RTP_qualcomm.launch](./src/gscam_RTP_pkg/src/examples/RTP_qualcomm.launch)
    
    After that, you can run the ROS node for an RTP qualcomm video stream with:
    ```
    source devel/setup.zsh
    roslaunch gscam RTP_qualcomm.launch
    ```


## Test

### One RTP connection Test

In order to test the gscam performance without connecting to a RTP address linked to a real camera, you can follow the next steps:

1. Create a RTP server linked to a `videotestsrc` in a local IP address with the encoder you prefer (h264 in this example).
    ```
    gst-launch-1.0 videotestsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2048 speed-preset=ultrafast ! h264parse config-interval=-1 ! rtph264pay ! udpsink host=127.0.0.1 port=5000
    ```

2. In another terminal, launch the node from our workspace:
    ```
    cd gscam_RTP
    source devel/setup.zsh
    roslaunch gscam RTP_to_ros.launch UDP_port:=5000 USE_H265:=false
    ```

3. In a new terminal, check if the images are being published
    ```
    rosrun image_view image_view image:=/RTP/camera/image_raw
    ```

### Two RTP connection Test

In order to test the gscam performance taking two simultaneoius video stream without connecting to a RTP address linked to a real camera, you can follow the next steps:

1. Create a RTP server linked to a `videotestsrc` in a local IP address with the encoder you prefer (h264 in this example).
    ```
    gst-launch-1.0 videotestsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2048 speed-preset=ultrafast ! h264parse config-interval=-1 ! rtph264pay ! udpsink host=127.0.0.1 port=5000
    ```

2. Create another RTP server linked to a `videotestsrc` in a local IP address with a different port with the encoder you prefer (h265 in this example).
    ```
    gst-launch-1.0 videotestsrc ! videoconvert ! x265enc tune=zerolatency bitrate=2048 speed-preset=ultrafast ! h265parse config-interval=-1 ! rtph265pay ! udpsink host=127.0.0.1 port=5001
    ```

3. In another terminal, launch the node from our workspace:
    ```
    cd gscam_RTP
    source devel/setup.zsh
    roslaunch gscam RTP_to_ros_two_cameras.launch UDP_port_cam1:=5000 USE_H265_cam1:=false UDP_port_cam2:=5001 USE_H265_cam2:=true
    ```

4. In a new terminal, check if the images from the first RTP communication are being published.
    ```
    rosrun image_view image_view image:=/RTP_1/camera/image_raw
    ```

5. After checking the first RTP connection, check if the images from the second RTP communication are being published.
    ```
    rosrun image_view image_view image:=/RTP_2/camera/image_raw
    ```

## Help / Troubleshooting

* Contact: **Marco A. Montes Grova** (mmontes@catec.aero) or **Antonio Luis González Hernández** (algonzalez.ext@catec.aero)

* Found a bug? Create an ISSUE!

* Do you want to contribute? Create a PULL-REQUEST!
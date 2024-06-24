# gscam for RTP connection

This submodule is a modified version of [gscam](https://github.com/ros-drivers/gscam).
The original version has two main issues: a problem with the Gstreamer version (More info: [#61](https://github.com/ros-drivers/gscam/pull/61)), and an issue when using an RTP port as a source in the pipeline provided through `gscam_config`. To address these issues, the following changes have been made compared to the original repository:

1. The change proposed in the following pull request has been applied: [#61](https://github.com/ros-drivers/gscam/pull/61)
2. In [gscam.cpp](src/src/gscam.cpp), the launchpipe element that contained the pipeline information described in `gscam_config` has been replaced with the combination of the created and configured pipeline elements within the same code. In this version of gscam **there is no need to pass any Gstreamer pipeline either from an environment variable or ROS params.**.
3. The variables and lines of code handling the `gscam_config` variable have been removed.
4. The ROS parameter `RTP_port` has been added to select which RTP port should be used to receive the images.
5. The ROS parameter `use_h265` has been added to select between h264 decoder or h265 decoder.
6. [RTP_to_ros.launch](src/examples/RTP_to_ros.launch) has been added to launch the node for a RTP connection.
7. [RTP_to_ros_two_cameras.launch](src/examples/RTP_to_ros_two_cameras.launch) has been added to launch the node for two RTP connection.

# Usage

You can build gscam with the following command:
```
catkin build -DGSTREAMER_VERSION_1_x=On -j4
```
Once built, the ROS node can be run using:
```
source devel/setup.zsh
roslaunch gscam RTP_to_ros.launch
```


# Test

In order to test the gscam performance without connecting to a RTP address linked to a real camera, you can follow the next steps:

1. We create a RTP server linked to a `videotestsrc` in a local IP address with a h264 encoder.
```
gst-launch-1.0 videotestsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2048 speed-preset=ultrafast ! h264parse config-interval=-1 ! rtph264pay ! udpsink host=127.0.0.1 port=5000
```

2. In another terminal, we launch the node from our workspace:
```
cd gscam_RTP
source devel/setup.zsh
roslaunch gscam RTP_to_ros.launch
```

3. In another terminal, we check if the images are being published
```
rosrun image_view image_view image:=/RTP/camera/image_raw
```
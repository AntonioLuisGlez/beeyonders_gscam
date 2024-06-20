# gscam for RTP connection

This submodule is a modified version of [gscam](https://github.com/ros-drivers/gscam). The original version has two main issues: a problem with the Gstreamer version (More info: [#61](https://github.com/ros-drivers/gscam/pull/61)), and an issue when using an RTP port as a source in the pipeline provided through `gscam_config`. To address these issues, the following changes have been made compared to the original repository:

1. The change proposed in the following pull request has been applied: [#61](https://github.com/ros-drivers/gscam/pull/61)
2. In [gscam.cpp](gscam_RTP/src/gscam.cpp), the launchpipe element that contained the pipeline information described in `gscam_config` has been replaced with the combination of the created and configured pipeline elements within the same code. In this version of gscam **there is no need to pass any Gstreamer pipeline either from an environment variable or ROS params.**.
3. The variables and lines of code handling the `gscam_config` variable have been removed.
4. The ROS parameter `RTP_port` has been added to select which RTP port should be used to receive the images.
5. [RTP_to_ros.launch](gscam_RTP/examples/RTP_to_ros.launch) has been added to launch the node.

# Usage

You can build gscam with the following command:
```
catkin build -DGSTREAMER_VERSION_1_x=On -j4
```
Once built, the ROS node can be run using:
```
cd ./gscam_RTP
roslaunch gscam RTP_to_ros.launch
```
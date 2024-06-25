#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String


def execute_command(command):
    try:
        result = subprocess.run(
            command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        rospy.loginfo("Command executed successfully: %s",
                      result.stdout.decode())
    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to execute command: %s", e.stderr.decode())


def remote_cmd_executor():
    rospy.init_node('remote_cmd_executor', anonymous=True)

    # Define your command here
    command = "gst-launch-1.0 -e \
v4l2src device=/dev/video0 ! video/x-raw,format=I420,width=640,height=514,framerate=60/1 ! queue ! x264enc tune=3 bitrate=10000 speed-preset=ultrafast ! h264parse config-interval=-1 ! rtph264pay ! udpsink host=10.42.0.1 port=5000 sync=false \
qtiqmmfsrc name=qmmf ! video/x-h265,format=NV12,width=1920,height=1080,framerate=30/1 ! h265parse config-interval=-1 ! rtph265pay ! udpsink host=10.42.0.1 port=5001 sync=false"

    rospy.loginfo("Executing command directly: %s", command)
    ssh_command = f"ssh username@qualcomm_ip '{command}'"
    execute_command(ssh_command)


if __name__ == '__main__':
    try:
        remote_cmd_executor()
    except rospy.ROSInterruptException:
        pass

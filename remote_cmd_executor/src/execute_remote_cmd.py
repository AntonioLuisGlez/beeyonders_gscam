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


def remote_cmd_executor(IR_width, IR_height, IR_framerate, IR_IP_address, IR_UDP_port,
                        IR_encoder_tune, IR_encoder_bitrate, IR_encoder_speed_preset,
                        EO_width, EO_height, EO_framerate, EO_IP_address, EO_UDP_port, username, qualcomm_ip):
    rospy.init_node('remote_cmd_executor', anonymous=True)

    # Define your command here
    command = f"gst-launch-1.0 -e \
v4l2src device=/dev/video0 ! video/x-raw,format=I420,width={IR_width},height={IR_height},framerate={IR_framerate} ! queue ! x264enc tune={IR_encoder_tune} bitrate={IR_encoder_bitrate} speed-preset={IR_encoder_speed_preset} ! h264parse config-interval=-1 ! rtph264pay ! udpsink host={IR_IP_address} port={IR_UDP_port} sync=false \
qtiqmmfsrc name=qmmf ! video/x-h265,format=NV12,width={EO_width},height={EO_height},framerate={EO_framerate} ! h265parse config-interval=-1 ! rtph265pay ! udpsink host={EO_IP_address} port={EO_UDP_port} sync=false"

    rospy.loginfo("Executing command directly: %s", command)
    ssh_command = f"ssh -tt {username}@{qualcomm_ip} '{command}'"
    execute_command(ssh_command)


if __name__ == '__main__':
    try:
        # Fetching parameters from ROS launch
        IR_width = rospy.get_param('/IR_width', '640')
        IR_height = rospy.get_param('/IR_height', '514')
        IR_framerate = rospy.get_param('/IR_framerate', '60/1')
        IR_IP_address = rospy.get_param('/IR_IP_address', '10.42.0.1')
        IR_UDP_port = rospy.get_param('/UDP_port_cam1', '5000')
        IR_encoder_tune = rospy.get_param('/IR_encoder_tune', '3')
        IR_encoder_bitrate = rospy.get_param('/IR_encoder_bitrate', '10000')
        IR_encoder_speed_preset = rospy.get_param(
            '/IR_encoder_speed-preset', 'ultrafast')

        EO_width = rospy.get_param('/EO_width', '1920')
        EO_height = rospy.get_param('/EO_height', '1080')
        EO_framerate = rospy.get_param('/EO_framerate', '30/1')
        EO_IP_address = rospy.get_param('/EO_IP_address', '10.42.0.1')
        EO_UDP_port = rospy.get_param('/UDP_port_cam2', '5001')

        username = rospy.get_param('/username', 'root')
        qualcomm_ip = rospy.get_param('/qualcomm_ip', '10.1.0.143')

        remote_cmd_executor(IR_width, IR_height, IR_framerate, IR_IP_address, IR_UDP_port,
                            IR_encoder_tune, IR_encoder_bitrate, IR_encoder_speed_preset,
                            EO_width, EO_height, EO_framerate, EO_IP_address, EO_UDP_port, username, qualcomm_ip)

    except rospy.ROSInterruptException:
        pass

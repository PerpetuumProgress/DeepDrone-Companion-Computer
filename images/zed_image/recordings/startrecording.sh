#!/bin/bash

echo "ROS source devel/setup.bash "
source ../ros_ws/devel/setup.bash
echo "Start the recording the nodes"
rosbag record -e "/zed2/zed_node/stereo/image_rect_color/compressed"


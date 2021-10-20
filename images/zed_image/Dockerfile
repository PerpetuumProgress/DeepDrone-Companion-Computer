FROM stereolabs/zed:3.4-ros-devel-jetson-jp4.3

RUN apt update && apt install -y ros-melodic-rviz ros-melodic-rqt ros-melodic-rqt-graph ros-melodic-rqt-common-plugins ros-melodic-rqt-tf-tree
RUN echo "source /opt/ros_ws/devel/setup.bash" >> ~/.bashrc
COPY *.conf /usr/local/zed/settings/

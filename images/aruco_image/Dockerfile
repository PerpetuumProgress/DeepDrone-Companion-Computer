FROM spokorny/l4t-ros_base

#switch keys
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA11
RUN sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#install dependencies
RUN apt clean && apt update && apt install -y \
	vim \
	ros-melodic-rviz \
	ros-melodic-rqt \
        ros-melodic-rqt-graph \
        ros-melodic-rqt-tf-tree \
        ros-melodic-rqt-common-plugins \
	ros-melodic-image-transport-plugins \
	ros-melodic-fiducials \
        ros-melodic-pcl-ros \
	ros-melodic-pcl-msgs \
	ros-melodic-pcl-conversions \
	ros-melodic-tf2-eigen \
	python-catkin-tools \
	libpcl-dev \
	&& rm -rf /var/lib/apt/lists/*

#create catkin workspace
RUN mkdir -p /root/catkin_ws/src

#copy vo_publisher project
COPY ./vo_publisher /root/catkin_ws/src/vo_publisher

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin build'

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

#setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


FROM spokorny/l4t-ros_base

#install dependencies
RUN apt update && apt install -y \
	python-catkin-tools \
	ros-melodic-mavros \
	ros-melodic-mavros-extras \
	libpcl1 \
	ros-melodic-octomap-* \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-ugly \
	libgstreamer-plugins-base1.0-dev \
	ros-melodic-stereo-image-proc \
	ros-melodic-image-view \
	ros-melodic-rqt \
	ros-melodic-rqt-graph \
	ros-melodic-rqt-tf-tree \
	ros-melodic-rqt-common-plugins \
  	&& rm -rf /var/lib/apt/lists/*

#install geographiclib
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

#create catkin workspace
RUN mkdir -p /root/catkin_ws/src

#Clone this repository in your catkin workspace in order to build the avoidance node.
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/PX4/avoidance.git

#Build the avoidance node
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin build -j2 -w ~/catkin_ws'

#Copy hitl launch files
COPY ./local_planner/launch/local_planner* /root/catkin_ws/src/avoidance/local_planner/launch/
COPY ./global_planner/launch/global_planner* /root/catkin_ws/src/avoidance/global_planner/launch/


RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]                 

#!/bin/bash
#
#


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo $DIR 


###############################################################################
# Help                                                                         #
################################################################################
Help()
{
   # Display Help
   echo "This script launches the avoidance code for px4 obstacle avoidance and collision prevention."
   echo
   echo "Syntax: run.sh -c [stereo_hitl|depth_hitl|stereo|ZED2] -p [local_planer|global_planner] [start|stop|status]"
   echo "options:"
   echo "-p     selects the planner you want to use: local_planner or global_planner"
   echo "-c     selects the camera you want to use: stereo_hitl, depth_hitl, stereo, ZED2"
   echo "start  starts the avoidance system"
   echo "stop   stops the avoidance system"
   echo "status Shows the status of the avoidance system"
}


################################################################################
# Dependency validation and initialisation                                     #
################################################################################

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


if ! command -v docker-compose
then
    echo "docker-compose could not be found. Installing Docker-Compose"

    DOCKER_COMPOSE_VERSION=1.27.4
    sudo apt-get install rustc curl libhdf5-dev python3 python3-pip python3-setuptools libhdf5-dev libffi-dev python-openssl libssl-dev zlib1g-dev gcc g++ make -y
    pip3 install --upgrade pip
    sudo pip3 install docker-compose=="${DOCKER_COMPOSE_VERSION}"
                       
else
   echo "docker-compose found"
fi


################################################################################
# Parameter validation                                                         #
################################################################################

#Get options
while getopts c:p: flag
do
    case "${flag}" in
        c) pCamera=${OPTARG};;
        p) pPlanner=${OPTARG};;
	*) echo "Supported options are:"
	   echo "-c selects the camera you want to use: stereo_hitl, depth_hitl, stereo, ZED2"
	   echo "-p selects the planner you want to use: local_planner or global_planner"
	   exit;;
    esac
done

#Validate user input for planner
case "${pPlanner}" in
	local_planner) planner="local_planner"
		echo "local planner is used";;
	global_planner) planner="global_planner"
		echo "global planned is used";; 
	*) echo "No valid planner type given. Please provide one of the following options:  local_planner or global_planner"; 
	   echo "local_planner will be used per default"
           planner="local_planner" 
esac


#Vaidate user input for camera
case "${pCamera}" in
	stereo_hitl) compose_path="$DIR/dc/stereo/simulation/docker-compose.yaml"
		echo "hitl stereo camera selected " ;;
	depth_hitl) compose_path="$DIR/dc/depth/simulation/docker-compose.yaml"
		echo "hitl depth camera selected" ;;
	zed2) compose_path="$DIR/dc/depth/hardware/zed2/docker-compose.yaml"
		echo "zed2 camera is selected" ;;
	stereo) compose_path="$DIR/dc/stereo/hardware/IMX219-83/docker-compose.yaml"
		echo "stereo camera IMX219-83 is selected" ;;
	*) echo "No valid camera information given. Please provide one of the following options: stereo_hitl, depth_hitl, stereo, ZED2"
	   echo "zed2 camera will be used per default"
           compose_path="$DIR/dc/depth/hardware/zed2/docker-compose.yaml"
esac



################################################################################
# start/stop services via docker-compose                                       #
################################################################################

for lastArgument; do true; done
case "$lastArgument" in
   start)
     if [[ -f $DIR/docker-compose.yaml ]]; then
       echo "Service is already running."
       echo "Stopping running service"
       docker-compose -f $DIR/docker-compose.yaml down
       unlink $DIR/docker-compose.yaml
     fi
     ln -s $compose_path $DIR/docker-compose.yaml
     PLANNER=$planner docker-compose -f $DIR/docker-compose.yaml up -d
     ;;
   stop)
     docker-compose -f $DIR/docker-compose.yaml down
     unlink $DIR/docker-compose.yaml
     ;;
   status)
     docker-compose -f $DIR/docker-compose.yaml ps
   ;;
   *)
     Help
     exit 1
     ;;
 esac



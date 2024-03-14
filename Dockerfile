# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

SHELL ["bash", "-c"]

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install MAVROS packages
RUN apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

# build ros dependency
RUN useradd -m -s /bin/bash random
WORKDIR /home/random/catkin_ws/src
RUN catkin_create_pkg mavrostutorial roscpp
COPY . /home/random/catkin_ws/src/mavrostutorial
RUN cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && source devel/setup.bash && catkin_make
RUN cd src

# install PX4 and dependencies
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# setup of PX4 on Ubuntu for simulation purposes
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# commands for PX4
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic
RUN source ~/catkin_ws/devel/setup.bash && \
    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
RUN export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
RUN export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# install geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN bash ./install_geographiclib_datasets.sh  

# # ready QGroundcontrol
# RUN chmod +x ./QGroundControl.AppImage  

##############################

# # start QGroundControl
# RUN ./QGroundControl.AppImage

# # launch MAVROS
# RUN roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# # launch PX4 and Gazebo
# RUN roslaunch px4 posix_sitl.launch

# # launch ROS Offboard Node
# RUN rosrun mavrostutorial node

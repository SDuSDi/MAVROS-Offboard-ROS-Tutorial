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
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

# build ros dependencies
RUN useradd -m -s /bin/bash catec
WORKDIR /home/catec/catkin_ws/src
RUN catkin_create_pkg mavrostutorial roscpp
COPY . /home/catec/catkin_ws/src/mavrostutorial
RUN cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make && source devel/setup.bash && cd src

# install vim, terminator, tmux and tmuxinator
RUN apt-get update && \
    apt-get install -y vim && \
    apt-get install -y tmux rubygems && \
    gem install thor -v 1.2.2 && \
    gem install tmuxinator -v 1.1.5

# install PX4 and dependencies
WORKDIR /home/catec
RUN apt install -y git-all && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# setup of PX4 on Ubuntu for simulation purposes
RUN bash /home/catec/PX4-Autopilot/Tools/setup/ubuntu.sh

# install geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh  

# install QGroundControl
RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage

# install QGroundControl dependencies
RUN apt-get install -y libpulse-mainloop-glib0

# build PX4
RUN cd /home/catec/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo-classic

##########################
# EJECUCIÓN DEL PROGRAMA #
##########################

# USER catec
ENV USER catec

# setup entrypoint
COPY ./entrypoint.sh /
COPY ./tmux /

ENTRYPOINT ["./catkin_ws/src/mavrostutorial/entrypoint.sh"]
CMD ["bash"]
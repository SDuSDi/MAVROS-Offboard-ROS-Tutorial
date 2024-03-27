# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

SHELL ["bash", "-c"]

ENV ROS_DISTRO noetic
RUN useradd -m -s /bin/bash catec
WORKDIR /home/catec

# instalation of prerequisites for MQTT
RUN apt-get update && \
    apt-get install -y build-essential gcc make cmake cmake-gui cmake-curses-gui && \
    apt-get install -y libssl-dev && \
    apt-get install -y doxygen graphviz && \
    apt install -y git-all

# install Paho MQTT C and build it
RUN cd /home/catec && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    git checkout v1.3.13 && \
    cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON && \
    cmake --build build/ --target install && \
    ldconfig

# install Paho MQTT C++ and build it
RUN cd /home/catec && \
    git clone https://github.com/eclipse/paho.mqtt.cpp && \
    cd paho.mqtt.cpp && \
    cmake -Bbuild -H. -DPAHO_WITH_MQTT_C=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=ON -DPAHO_BUILD_SAMPLES=ON && \
    cmake --build build/ --target install && \
    ldconfig

# install nlohmann JSON library
RUN apt-get update && apt-get install nlohmann-json3-dev

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install MAVROS packages
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

# install vim, terminator, tmux and tmuxinator
RUN apt-get update && \
    apt-get install -y vim && \
    apt-get install -y tmux rubygems && \
    gem install thor -v 1.2.2 && \
    gem install tmuxinator -v 1.1.5

# install PX4 and dependencies
WORKDIR /home/catec
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# setup of PX4 on Ubuntu for simulation purposes
RUN bash /home/catec/PX4-Autopilot/Tools/setup/ubuntu.sh

# install geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh  

# # install QGroundControl
# RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage && \
#     chmod +x QGroundControl.AppImage

# # install QGroundControl dependencies
# RUN apt-get install -y libpulse-mainloop-glib0

# build PX4
RUN cd /home/catec/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo-classic

# build ros dependencies
WORKDIR /home/catec/catkin_ws/src
RUN catkin_create_pkg mavrostutorial roscpp
COPY . /home/catec/catkin_ws/src/mavrostutorial
RUN cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make && source devel/setup.bash && cd src


##########################
# EJECUCIÓN DEL PROGRAMA #
##########################

WORKDIR /home/catec

# USER catec
ENV USER catec

# setup entrypoint
COPY ./entrypoint.sh /
COPY ./tmux /

ENTRYPOINT ["./catkin_ws/src/mavrostutorial/entrypoint.sh"]
CMD ["bash"]
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/home/$USER/catkin_ws/devel/setup.bash"

# launch PX4 and Gazebo
cd /home/catec/PX4-Autopilot && \
DONT_RUN=1 make px4_sitl_default gazebo-classic && \
source "/home/catec/catkin_ws/devel/setup.bash" && \
source /home/catec/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$(pwd)" && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

# start tmuxinator
cd /home/catec/catkin_ws/src/mavrostutorial/ && \
tmuxinator start -p ./tmux/session.yml
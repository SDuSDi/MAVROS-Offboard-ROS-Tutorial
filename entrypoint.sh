#!/bin/bash
set -e

touch ~/.tmux.conf
echo "set -g mouse" >> ~/.tmux.conf

/home/catec/catkin_ws/src/mavrostutorial/tmux/start.sh
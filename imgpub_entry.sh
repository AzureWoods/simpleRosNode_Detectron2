#!/bin/bash
set -e

source /opt/ros/eloquent/setup.bash; cd /home/dev_ws; source ./install/local_setup.bash; ros2 run imgpub_pkg imgpub
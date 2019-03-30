#!/bin/bash

set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/jason_ws/devel/setup.bash"

exec "$@"

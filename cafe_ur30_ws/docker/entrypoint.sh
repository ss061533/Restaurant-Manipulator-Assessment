#!/bin/bash
# Source every overlay that exists, then run the given command.
set -e
source /opt/ros/humble/setup.bash
[ -f /opt/ur_ws/install/setup.bash ] && source /opt/ur_ws/install/setup.bash
if [ -f /opt/gz_ws/install/setup.bash ]; then
  source /opt/gz_ws/install/setup.bash
  export GZ_VERSION=harmonic
  export CAFE_GZ=true          # cafe.launch.xml then starts the new Gazebo (gz) by default
fi
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
exec "$@"

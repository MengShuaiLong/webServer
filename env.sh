#!/bin/bash

if [ -f  /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f  /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f  /opt/ros/humble/install/setup.bash ]; then
    source /opt/ros/humble/install/setup.bash
fi
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi
export ROS_LOG_DIR=$PWD/runlog
export ROS_HOME=$PWD
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}] ({function_name}:{line_number}) :{message}"

export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/lib/aarch64-linux-gnu/pkgconfig

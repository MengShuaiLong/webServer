#!/bin/bash

local_path=$(dirname $(readlink -f "$0"))

cd $local_path
echo $local_path
source env.sh

ros2 run webServer server
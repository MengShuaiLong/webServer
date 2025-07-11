#!/bin/bash

local_path=$(dirname $(readlink -f "$0"))

cd $local_path
echo $local_path
source env.sh

MSG_TEMPLATE='header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
speed: 0.02
gear: 9
mode: 0
version: '1.2.3'
file_name: '''

IMEI_BASE="dh129"

for i in $(seq -w 0 49); do
    FULL_MSG="$MSG_TEMPLATE
imei: '$IMEI_BASE_$i'"
    
    ros2 topic pub -1 /communication/cloud/online_truck common_msgs/msg/VehicleInfo "$FULL_MSG"
    
    echo "已发布消息 #$i: imei=$IMEI_BASE_$i"
    
    sleep 0.1
done

echo "完成100次消息发布"    

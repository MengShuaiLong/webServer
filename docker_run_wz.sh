#!/bin/bash
docker_name="lk2"
local_path=$(cd "$(dirname "$0")";pwd)
docker_path="/${local_path##*/}"

export SUDO_ASKPASS=$local_path/SUDO_PWD_ACU_

if [[ -n $(sudo -A docker ps -a -q -f "name=^$docker_name$") ]];then
    echo -e "\n[$(date +"%Y-%m-%d %H:%M:%S")] stop old container(name:$docker_name)"
    sudo -A docker stop $docker_name
    sleep 1
    sudo -A docker rm $docker_name
    sleep 1
fi

xhost +;
sudo -A docker run   -it --privileged \
-e DISPLAY=$DISPLAY \
--net=host \
--name=$docker_name \
--workdir $docker_path \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
-v /home/:/home/ \
-w $local_path \
registry.cn-beijing.aliyuncs.com/rockai-hub/qt5.15-ros-humble-desktop-ubuntu22.04-x64:latest  \
/bin/bash

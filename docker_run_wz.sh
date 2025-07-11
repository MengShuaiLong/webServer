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

sudo -A docker run -it \
	-e DISPLAY=$DISPLAY \
	--net=host --pid=host \
	--rm -it \
	--name $docker_name  \
	--runtime nvidia \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v /etc/localtime:/etc/localtime:ro \
	-v /home/nvidia:/home/nvidia \
	-v /nvmedisk:/nvmedisk \
	--workdir $PWD \
	172.28.1.153:5000/ros2-humble-35.1.0:latest /bin/bash



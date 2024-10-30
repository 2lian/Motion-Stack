#!/bin/bash

xhost +local:root

# Create Docker volumes if they don't already exist
docker volume create moonbot_build
docker volume create moonbot_install
docker volume create moonbot_log

docker run -it --rm \
    --name="moonbot-motion-stack" \
    --network=host --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/moonbot" \
    --volume="moonbot_build:/moonbot/build" \
    --volume="moonbot_install:/moonbot/install" \
    --volume="moonbot_log:/moonbot/log" \
    moonbot

xhost -local:root
#!/bin/bash

xhost +local:root

docker run -it --rm \
    --name="moonbot-motion-stack" \
    --network=host --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/moonbot" \
    --volume="$(pwd)/docker/build:/moonbot/build" \
    --volume="$(pwd)/docker/install:/moonbot/install" \
    --volume="$(pwd)/docker/log:/moonbot/log" \
    moonbot

xhost -local:root
#!/bin/bash

xhost +local:root

docker run -it --rm \
    --name="moonbot-motion-stack" \
    --network=host --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/moonbot:ro" \
    --mount type=tmpfs,destination=/moonbot/build \
    --mount type=tmpfs,destination=/moonbot/install \
    --mount type=tmpfs,destination=/moonbot/log \
    moonbot

xhost -local:root
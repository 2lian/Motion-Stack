#!/bin/bash

docker run -it --rm \
    --name="moonbot-motion-stack" \
    --network=host --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/moonbot" \
    --volume="$(pwd)/docker/build:/root/moonbot/build" \
    --volume="$(pwd)/docker/install:/root/moonbot/install" \
    --volume="$(pwd)/docker/log:/root/moonbot/log" \
    moonbot
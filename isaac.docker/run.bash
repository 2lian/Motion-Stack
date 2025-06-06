#!/bin/bash

xhost +local:root

# For more info, see: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-deployment
ISAAC_OPTIONS=(
    --runtime=nvidia 
    --gpus all 
    -e "ACCEPT_EULA=Y" 
    -e "PRIVACY_CONSENT=Y"
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw
    -v ~/docker/isaac-sim/documents:/root/Documents:rw
)

DOCKER_OPTIONS=(
    -it --rm
    --name="moonbot-motion-stack"
    --network=host --ipc=host
    --env="DISPLAY"
    --env="QT_X11_NO_MITSHM=1"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
)


SCRIPT_ARGS=()
CMD_ARGS=() # Arguments to pass to the command (everything after --)

# Use a flag to detect when we've hit --
FOUND_SEPARATOR=false

for arg in "$@"; do
    if [ "$arg" == "--" ]; then
        FOUND_SEPARATOR=true
        continue
    fi
    if [ "$FOUND_SEPARATOR" == false ]; then
        SCRIPT_ARGS+=("$arg")
    else
        CMD_ARGS+=("$arg")
    fi
done
    

# Instead of mounting the whole workspace, mount only the frequently 
# changed files and folders. This is necessary because by default, 
# files generated in the container (ie. build/ install/ log/) are owned 
# by root which makes it cumbersome to modify/delete them on the host system.
# A possible cleaner solution would be to use the --build-base and --install-base 
# colcon arguments to move the docker generated build artifacts to another location

# Mount src/ directory as read-only
DOCKER_OPTIONS+=(
    --mount type=bind,source="$(pwd)/src",destination=/moonbot/src,readonly \
)

# Mount all shell scripts and python files in the root of the project directory as read-only
for file in *.sh *.py *.bash; do
    if [[ -f $file ]]; then
        DOCKER_OPTIONS+=(
            --mount type=bind,source="$(pwd)/$file",destination=/moonbot/"$file",readonly
        )
    fi
done

# If no CMD_ARGS were provided, default to bash
if [ ${#CMD_ARGS[@]} -eq 0 ]; then
    CMD_ARGS=("/bin/bash")
fi

docker run \
    "${DOCKER_OPTIONS[@]}" \
    "${ISAAC_OPTIONS[@]}" \
    moonbot \
    "${CMD_ARGS[@]}"

xhost -local:root
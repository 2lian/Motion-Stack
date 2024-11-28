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
    
# Mount source code if --dev flag is passed
for arg in "$@"; do
    if [ "$arg" == "--dev" ]; then
        # Mount the whole project directory as read-only and create tmpfs volumes for build artifacts
        DOCKER_OPTIONS+=(
            --mount type=bind,source="$(pwd)",destination=/moonbot \
        )

        # Mount all shell scripts and python files in the project directory as read-only
        for file in *.sh *.py *.bash; do
            if [[ -f $file ]]; then
                DOCKER_OPTIONS+=(
                    --mount type=bind,source="$(pwd)/$file",destination=/moonbot/"$file",readonly
                )
            fi
        done
        break
    fi
done

docker run \
    "${DOCKER_OPTIONS[@]}" \
    "${ISAAC_OPTIONS[@]}" \
    moonbot \
    /bin/bash

xhost -local:root
#!/bin/bash
DOCKER_OPTIONS=(
    --mount type=bind,source="$(pwd)/README.md",destination=/moonbot/README.md,readonly \
)
#!/bin/bash
set -e -o pipefail
source ~/.bashrc

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

# SSH_ADDRESS=hero-arm-v1-1
# SSH_ADDRESS=hero-arm-v1-2
# SSH_ADDRESS=hero-arm-v1-3
# SSH_ADDRESS=hero-arm-v1-4
# SSH_ADDRESS=hero-arm-v2-1
# SSH_ADDRESS=hero-arm-v2-2
# SSH_ADDRESS=hero-arm-v3-1
# SSH_ADDRESS=hero-arm-v3-2
SSH_ADDRESS=hero-arm-v3-3
# SSH_ADDRESS=hero-wheel-v1-1
# SSH_ADDRESS=hero-wheel-v1-2
# SSH_ADDRESS=hero-wheel-v1-3
# SSH_ADDRESS=hero-wheel-v3-4
# SSH_ADDRESS=hero-wheel-v2-1
# SSH_ADDRESS=hero-wheel-v2-2
# SSH_ADDRESS=hero-wheel-v3-1
# SSH_ADDRESS=hero-wheel-v3-2
SOURCE_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"/
DESTINATION_DIR="${SSH_ADDRESS}:~/moonbot_tools/cache/motion_stack/"


rsync -av --checksum --progress --exclude='*cache*' --exclude='.*' --include='***' $SOURCE_DIR $DESTINATION_DIR


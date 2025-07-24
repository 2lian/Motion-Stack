#!/bin/bash
set -e -o pipefail
source ~/.bashrc

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

# SSH_ADDRESS=hero-arm-v1-1
SSH_ADDRESS=moonshot@192.168.8.130
SOURCE_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
DESTINATION_DIR="${SSH_ADDRESS}:~/moonbot_tools/cache/motion_stack/"

# rsync -av --checksum --progress --exclude='build/'--exclude='install/'  --exclude='log/' --exclude='*cache*' $SOURCE_DIR $DESTINATION_DIR

rsync -av --checksum --progress --exclude='*cache*' --exclude='.*' --include='***' $SOURCE_DIR $DESTINATION_DIR


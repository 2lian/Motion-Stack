#!/bin/bash
set -e -o pipefail
source ~/.bashrc

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

SSH_ADDRESS=hero-arm-v1-1
SOURCE_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
DESTINATION_DIR="${SSH_ADDRESS}:~/moonbot_tools/cache/"

# rsync -av --checksum --progress --exclude='build/'--exclude='install/'  --exclude='log/' --exclude='*cache*' $SOURCE_DIR $DESTINATION_DIR

rsync -av --checksum --progress --exclude='*cache*' --exclude='.*' --exclude='hero_workspace' --include='***' $SOURCE_DIR $DESTINATION_DIR


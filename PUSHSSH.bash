#!/bin/bash
set -e -o pipefail
source ~/.bashrc

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

SSH_ADDRESS=${m1}
SOURCE_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
DESTINATION_DIR="${SSH_ADDRESS}:~/Motion-Stack/"

# rsync -av --checksum --progress --exclude='build/'--exclude='install/'  --exclude='log/' --exclude='*cache*' $SOURCE_DIR $DESTINATION_DIR

rsync -av --checksum --progress --exclude='.*.db' --exclude='*.vim' --exclude='*cache*' --include='src/***' --include='*.*' --exclude='*' $SOURCE_DIR $DESTINATION_DIR
rsync -av --checksum --progress --exclude='.*.db' --exclude='*.vim' --include='*.bash' --include='*.py' --exclude='*' $SOURCE_DIR $DESTINATION_DIR
echo 'DONE'

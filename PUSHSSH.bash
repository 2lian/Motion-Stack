#!/bin/bash
set -e -o pipefail

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

SSH_ADDRESS=${address_eli}
SOURCE_DIR="./"
DESTINATION_DIR="${SSH_ADDRESS}:~/Moonbot-Motion-Stack/"

rsync -av --progress --include='src/***' --include='*.*' --exclude='*' $SOURCE_DIR $DESTINATION_DIR
rsync -av --progress --include='*.bash' --include='*.py' --exclude='*' $SOURCE_DIR $DESTINATION_DIR



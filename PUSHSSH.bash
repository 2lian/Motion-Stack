#!/bin/bash
set -e -o pipefail

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

SSH_ADDRESS=${address_lap1}
SOURCE_DIR="./"
DESTINATION_DIR="${SSH_ADDRESS}:~/elian/motion_stack_ssh/"

rsync -av --progress --include='src/***' --include='*.*' --exclude='*' $SOURCE_DIR $DESTINATION_DIR



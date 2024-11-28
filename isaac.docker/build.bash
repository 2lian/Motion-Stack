#!/bin/bash

# Build the Docker image
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
TAG="moonbot"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"
USER_ID=$(id -u)

# Use the parent directory as the build context
DOCKER_BUILD_CMD=(docker build -f "${DOCKERFILE}" "${PROJECT_DIR}" --tag ${TAG} --build-arg user_id=$USER_ID)

echo -e "\033[0;32m${DOCKER_BUILD_CMD[*]}\033[0m" | xargs

# shellcheck disable=SC2068
exec ${DOCKER_BUILD_CMD[*]}
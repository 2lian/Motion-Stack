#!/bin/bash

# Delete ROS build artifacts
docker volume rm moonbot_build
docker volume rm moonbot_install
docker volume rm moonbot_log
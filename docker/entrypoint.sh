#!/bin/bash

echo "Starting Gazebo Harmonic environment..."

# export GZ_SIM_RESOURCE_PATH=/workspace/models:/workspace/worlds:/usr/share/gz
export GZ_SIM_RESOURCE_PATH=/workspace/models:/workspace/worlds:/usr/share/gz:/workspace/Projects/gazebo_demo/models:/workspace/Projects/gazebo_demo/worlds

exec "$@"

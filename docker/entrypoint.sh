#!/bin/bash

echo "Starting Gazebo Harmonic environment..."

export GZ_SIM_RESOURCE_PATH=/workspace/models

exec "$@"

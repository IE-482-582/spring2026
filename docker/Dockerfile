FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic
RUN apt update && apt install -y \
    curl gnupg lsb-release \
    python3 python3-pip \
    libgl1-mesa-glx \
    && curl -sSL http://packages.osrfoundation.org/gazebo.gpg | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt update \
    && apt install -y gz-harmonic python3-gz-transport13 \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install numpy opencv-python

# Environment
ENV GZ_SIM_RESOURCE_PATH=/workspace/models
ENV DISPLAY=:0

WORKDIR /workspace

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

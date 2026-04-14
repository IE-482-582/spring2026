FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    software-properties-common \
    python3 \
    python3-pip \
    libgl1-mesa-dri \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo Harmonic repo (NEW FORMAT)
RUN curl -sSL https://packages.osrfoundation.org/gz.key | gpg --dearmor > /usr/share/keyrings/gz-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/gz-archive-keyring.gpg] http://packages.osrfoundation.org/gz/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gz-stable.list

RUN apt update && apt install -y \
    gz-harmonic \
    python3-gz-transport13 \
    && rm -rf /var/lib/apt/lists/*

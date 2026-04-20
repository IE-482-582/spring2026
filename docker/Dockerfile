FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic
RUN apt update && apt install -y \
    curl gnupg lsb-release \
    python3 python3-pip python3-numpy python3-opencv \
    libgl1 \
    && curl -sSL http://packages.osrfoundation.org/gazebo.gpg | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt update \
    && apt install -y gz-harmonic python3-gz-transport13 \
    && rm -rf /var/lib/apt/lists/*

# Text editor
RUN apt update && apt install -y nano

# Environment
ENV DISPLAY=:0

COPY Projects/socket_demo/client/requirements.txt /tmp/req_socket.txt
COPY Projects/ub_racer/client/requirements.txt /tmp/req_racer.txt
RUN pip install --no-cache-dir --break-system-packages -r /tmp/req_socket.txt -r /tmp/req_racer.txt

WORKDIR /workspace

RUN apt install git -y

# ub_camera pre-reqs and ub_code
# numpy/ultralytics first, then swap opencv packages, then remaining deps
RUN pip install --break-system-packages --ignore-installed \
        "numpy>=1.21.0" \
        "ultralytics>=8.3.0" \
    && pip uninstall --break-system-packages -y opencv-python opencv-contrib-python \
    && pip install --break-system-packages \
        "opencv-contrib-python>=4.10.0" \
        "pyzbar>=0.1.9" \
        "websockets>=12.0" \
        "aiortc>=1.9.0" \
        "aiohttp>=3.9.0" \
        git+https://github.com/optimatorlab/ub_code.git@main


COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

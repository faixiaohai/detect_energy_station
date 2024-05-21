FROM ros:iron-ros-base

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

WORKDIR /detect_buff

RUN apt-get update && \
    apt-get install wget curl git cmake ninja-build kmod ros-iron-foxglove-bridge -y
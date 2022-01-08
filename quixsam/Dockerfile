FROM ubuntu

LABEL description="Container for GTSAM Python Development"

ARG DEBIAN_FRONTEND=noninteractive

## Update packages
RUN apt-get update -q

## Install packages
RUN apt-get install -y --fix-missing \
    # Package Manager
    software-properties-common \
    # Basic Requirments
    build-essential cmake git python3-pip \
    # GTSAM Dependancies
    libboost-all-dev libtbb-dev

## Install Python dependancies
COPY requirements.txt /
RUN pip3 install -r requirements.txt

## Install GTSAM
WORKDIR /tmp
RUN git clone https://github.com/borglab/gtsam.git --branch develop && \
    pip3 install -r gtsam/python/requirements.txt && \
    cd gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_BUILD_PYTHON=2 \
          -DGTSAM_PYTHON_VERSION=3.8 \
          .. && \
    make check -j8&& \
    make python-install -j8
# Updating the prebuilt version (needs access to docker.io/mainiomano/bananas-mavsdk):
# VERSION=0.0.1
# podman build -t bananas-mavsdk:"$VERSION" . && podman push bananas-mavsdk:"$VERSION" docker://docker.io/mainiomano/bananas-mavsdk:"$VERSION"
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    git \
    tar \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    # Dependencies
    libcurl4-openssl-dev \
    libjsoncpp-dev \
    liblzma-dev \
    libtinyxml2-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/mavlink-src

# Download MAVLink and MAVSDK.
ADD https://github.com/mavlink/mavlink/archive/5e3a42b8f3f53038f2779f9f69bd64767b913bb8.tar.gz mavlink.tar.gz
RUN tar -xzf mavlink.tar.gz
ADD https://github.com/ardupilot/pymavlink/archive/533161c47995ba47f5548636eed37a47e91072c0.tar.gz pymavlink.tar.gz
RUN tar -xzf pymavlink.tar.gz -C mavlink-5e3a42b8f3f53038f2779f9f69bd64767b913bb8/pymavlink --strip-components 1
# MAVSDK insists on using a git clone.
RUN git clone --branch v3.2.0 --depth 1 --recursive https://github.com/mavlink/MAVSDK

# Build MAVLink.
WORKDIR /usr/mavlink-src/mavlink-5e3a42b8f3f53038f2779f9f69bd64767b913bb8
RUN cmake -GNinja -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/mavlink
RUN cmake --build build --target install

# Build MAVSDK.
WORKDIR /usr/mavlink-src/MAVSDK
RUN cmake -GNinja -S . -B build \
    -DBUILD_TESTING=OFF \
    -DSUPERBUILD=OFF \
    -DCMAKE_PREFIX_PATH=/usr/mavlink \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/mavlink
RUN cmake --build build --target install

WORKDIR /usr/mavlink

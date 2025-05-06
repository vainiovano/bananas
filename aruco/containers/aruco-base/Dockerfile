# Dockerfile for downloading dependencies of the ArUco detector. This image is
# useful both for local testing and as a basis for running the CI on.
#
# Updating for CI (needs access to docker.io/mainiomano/bananas-base):
# VERSION=0.0.4
# podman build -t bananas-base:"$VERSION" . && podman push bananas-base:"$VERSION" docker://docker.io/mainiomano/bananas-base:"$VERSION"

FROM ubuntu:24.04

ADD --chown=_apt https://packages.osrfoundation.org/gazebo.gpg /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
ADD --chown=_apt https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    tar \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    # OGRE dependencies \
    libsdl2-2.0.0 \
    # OpenCV dependencies
    libavcodec60 \
    libavformat60 \
    libavutil58 \
    libswscale7 \
    libqt6core6t64 \
    libqt6gui6t64 \
    libqt6test6t64 \
    libqt6widgets6t64 \
    # Direct dependencies
    libeigen3-dev \
    nlohmann-json3-dev \
    libtinyxml2-dev \
    # Gazebo
    gz-harmonic \
    # ROS
    ros-jazzy-image-transport \
    ros-jazzy-ros-base \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies built in other containers.
COPY --from=docker.io/mainiomano/bananas-opencv:0.0.2 /usr/opencv /usr/opencv
COPY --from=docker.io/mainiomano/bananas-ogre:0.0.1 /usr/ogre /usr/ogre
COPY --from=docker.io/mainiomano/bananas-mavsdk:0.0.1 /usr/mavlink /usr/mavlink
COPY --from=docker.io/mainiomano/bananas-px4:0.0.1 /usr/px4 /usr/px4

# Build ROS cv_bridge for our OpenCV version.
WORKDIR /usr/vision_opencv-src

ADD https://github.com/ros-perception/vision_opencv/archive/refs/tags/4.1.0.tar.gz vision_opencv-4.1.0.tar.gz
RUN tar -xzf vision_opencv-4.1.0.tar.gz

WORKDIR /usr/vision_opencv-src/vision_opencv-4.1.0/cv_bridge

RUN . /opt/ros/jazzy/local_setup.sh \
    && cmake -GNinja -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/usr/opencv \
    -DCV_BRIDGE_DISABLE_PYTHON=ON \
    -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/cv_bridge
RUN cmake --build build --target install

WORKDIR /usr

# Let ld.so find our prebuilt dependencies.
RUN echo /usr/opencv/lib > /etc/ld.so.conf.d/bananas.conf
RUN echo /usr/ogre/lib >> /etc/ld.so.conf.d/bananas.conf
RUN echo /usr/cv_bridge/lib >> /etc/ld.so.conf.d/bananas.conf
RUN ldconfig

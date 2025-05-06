# Updating the prebuilt version (needs access to docker.io/mainiomano/bananas-px4):
# VERSION=0.0.1
# podman build -t bananas-px4:"$VERSION" . && podman push bananas-px4:"$VERSION" docker://docker.io/mainiomano/bananas-px4:"$VERSION"

FROM ubuntu:24.04

ADD --chown=_apt https://packages.osrfoundation.org/gazebo.gpg /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    git \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    # Dependencies
    cppzmq-dev \
    gz-harmonic \
    libgstreamer-plugins-base1.0-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies built in other containers.
COPY --from=bananas-opencv:0.0.2 /usr/opencv /usr/opencv

# Build PX4.
WORKDIR /usr/px4-src
RUN git clone --branch v1.16.0-beta1 --depth 1 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /usr/px4-src/PX4-Autopilot

RUN python3 -m pip install --break-system-packages -r ./Tools/setup/requirements.txt

RUN cmake -GNinja -S . -B build \
    -DPX4_CONFIG=px4_sitl_default \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/usr/opencv \
    -DBUILD_TESTING=OFF \
    # For some reason, The PX4 binary doesn't link if it's installed to /usr.
    # Install it to an alternative path.
    -DCMAKE_INSTALL_PREFIX=/usr/px4
RUN cmake --build build --target install

WORKDIR /usr/px4

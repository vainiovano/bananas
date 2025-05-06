# Dockerfile for building a recent version of OGRE on Ubuntu 24.04.
#
# Updating the prebuilt version (needs access to docker.io/mainiomano/bananas-ogre):
# VERSION=0.0.1
# podman build -t bananas-ogre:"$VERSION" . && podman push bananas-ogre:"$VERSION" docker://docker.io/mainiomano/bananas-ogre:"$VERSION"

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    tar \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    # Dependencies
    libxrandr-dev \
    libsdl2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/ogre-src

# Download OGRE.
ADD https://github.com/OGRECave/ogre/archive/refs/tags/v14.3.4.tar.gz ogre-14.3.4.tar.gz
RUN tar -xzf ogre-14.3.4.tar.gz

WORKDIR /usr/ogre-src/ogre-14.3.4

# Build OGRE.
RUN cmake -GNinja -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DOGRE_BUILD_DEPENDENCIES=OFF \
    -DOGRE_BUILD_SAMPLES=OFF \
    -DOGRE_BUILD_COMPONENT_BITES=ON \
    -DOGRE_BUILD_COMPONENT_MESHLODGENERATOR=OFF \
    -DOGRE_BUILD_COMPONENT_PAGING=OFF \
    -DOGRE_BUILD_COMPONENT_PROPERTY=OFF \
    -DOGRE_BUILD_COMPONENT_TERRAIN=OFF \
    -DOGRE_BUILD_COMPONENT_VOLUME=OFF \
    -DOGRE_BUILD_PLUGIN_BSP=OFF \
    -DOGRE_BUILD_PLUGIN_OCTREE=OFF \
    -DOGRE_BUILD_PLUGIN_PCZ=OFF \
    -DOGRE_BUILD_PLUGIN_PFX=OFF \
    -DOGRE_BUILD_RENDERSYSTEM_GL=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/ogre
RUN cmake --build build --target install

WORKDIR /usr/ogre

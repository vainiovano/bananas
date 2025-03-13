FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ADD https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb /usr/pkg/cuda-keyring_1.1-1_all.deb
RUN dpkg -i /usr/pkg/cuda-keyring_1.1-1_all.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    tar \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    pkg-config \
    # Dependencies
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    qt6-base-dev \
    # CUDA
    cuda-compiler-12-8 \
    cuda-cudart-dev-12-8 \
    libcublas-dev-12-8 \
    libcufft-dev-12-8 \
    libnpp-dev-12-8 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/opencv-src

# Download OpenCV.
ADD https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz opencv-4.10.0.tar.gz
ADD https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.tar.gz opencv_contrib-4.10.0.tar.gz
RUN tar -xzf opencv-4.10.0.tar.gz
RUN tar -xzf opencv_contrib-4.10.0.tar.gz

WORKDIR /usr/opencv-src/opencv-4.10.0

# Build OpenCV.
RUN cmake -GNinja -S . -B build \
#    -DCMAKE_C_COMPILER=gcc-12 \
#    -DCMAKE_CXX_COMPILER=g++-12 \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.10.0/modules \
    -DBUILD_LIST=core,cudev,objdetect,videoio,highgui,calib3d,imgcodecs \
    -DWITH_CUDA=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_QT=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/opencv
RUN cmake --build build --target install

WORKDIR /usr/opencv

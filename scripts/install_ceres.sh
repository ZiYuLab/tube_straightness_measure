#!/bin/bash
set -e

# Install Ceres Solver from source (stable 2.x)
CERES_VERSION="2.2.0"
BUILD_DIR="/tmp/ceres_build"

sudo apt-get install -y \
  cmake \
  libgoogle-glog-dev \
  libgflags-dev \
  libatlas-base-dev \
  libeigen3-dev \
  libsuitesparse-dev

mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
wget -q "http://ceres-solver.org/ceres-solver-${CERES_VERSION}.tar.gz"
tar xzf "ceres-solver-${CERES_VERSION}.tar.gz"
mkdir build && cd build
cmake "../ceres-solver-${CERES_VERSION}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF
make -j"$(nproc)"
sudo make install
sudo ldconfig

rm -rf "$BUILD_DIR"
echo "Ceres ${CERES_VERSION} installed."

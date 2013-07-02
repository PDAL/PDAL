#!/bin/bash
# Installs requirements for PDAL
source ./scripts/ci/common.sh
sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192 || exit 1
sudo apt-get install python-software-properties -y || exit 1
sudo add-apt-repository ppa:ubuntugis/ppa -y || exit 1
sudo apt-get update -qq || exit 1
sudo apt-get install \
    cmake \
    libflann-dev \
    libgdal-dev \
    libpq-dev \
    libproj-dev \
    libtiff4-dev \
    libxml2-dev \
    || exit 1

# install libgeotiff from sources
wget http://download.osgeo.org/geotiff/libgeotiff/libgeotiff-1.4.0.tar.gz || exit 1
tar -xzf libgeotiff-1.4.0.tar.gz || exit 1
cd libgeotiff-1.4.0 || exit 1
./configure --prefix=/usr && make && sudo make install || exit 1
cd $TRAVIS_BUILD_DIR

gcc --version || exit 1
clang --version || exit 1
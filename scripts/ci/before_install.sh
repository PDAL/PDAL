#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh
sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192
sudo apt-get update -y
sudo apt-get install software-properties-common -y
sudo apt-get install python-software-properties -y
sudo add-apt-repository ppa:ubuntugis/ppa -y
sudo add-apt-repository ppa:boost-latest/ppa -y
sudo apt-get update -qq
sudo apt-get install \
    cmake \
    libflann-dev \
    libgdal-dev \
    libgeos-dev \
    libgeos++-dev \
    libpq-dev \
    libproj-dev \
    libtiff4-dev \
    libxml2-dev \
    python-numpy \
    boost1.55 -y

# install libgeotiff from sources
wget http://download.osgeo.org/geotiff/libgeotiff/libgeotiff-1.4.0.tar.gz
tar -xzf libgeotiff-1.4.0.tar.gz
cd libgeotiff-1.4.0
./configure --prefix=/usr && make && sudo make install
cd $TRAVIS_BUILD_DIR

gcc --version
clang --version

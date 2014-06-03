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
    boost1.55

if [ "$PDAL_OPTIONAL_COMPONENTS" == "all" ]
then
    sudo apt-get install \
        libgdal-dev \
        libgeos-dev \
        libgeos++-dev \
        libpq-dev \
        libproj-dev \
        python-numpy \
        libxml2-dev \
        libflann-dev \
        libtiff4-dev

    # install libgeotiff from sources
    wget http://download.osgeo.org/geotiff/libgeotiff/libgeotiff-1.4.0.tar.gz
    tar -xzf libgeotiff-1.4.0.tar.gz
    cd libgeotiff-1.4.0
    ./configure --prefix=/usr && make && sudo make install
    cd $TRAVIS_BUILD_DIR
fi

gcc --version
clang --version

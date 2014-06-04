#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh
sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192
sudo apt-get update -y

sudo apt-get install software-properties-common -y
sudo apt-get install python-software-properties -y

sudo add-apt-repository ppa:ubuntugis/ppa -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
# Use embedded boost for clang builds
# pdal_test segfaults when built against external g++-built boost,
# and I haven't found a good boost package built with clang yet
if [[ "$CXX" == "g++" ]]
then
    sudo add-apt-repository ppa:boost-latest/ppa -y
fi
sudo apt-get update -qq

# Install g++-4.8 (even if we're building clang) for updated libstdc++
sudo apt-get install \
    cmake \
    g++-4.8
if [[ "$CXX" == "g++" ]]
then
    sudo apt-get install boost1.55
fi

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

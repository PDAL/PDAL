#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh

sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192
sudo mv /etc/apt/sources.list.d/pgdg-source.list* /tmp
sudo apt-get -qq remove postgis
sudo apt-get update -qq
sudo apt-get install software-properties-common -y
sudo apt-get install python-software-properties -y
# sudo add-apt-repository ppa:ubuntugis/ppa -y
sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo add-apt-repository ppa:boost-latest/ppa -y
sudo apt-get update -qq

# Install g++-4.8 (even if we're building clang) for updated libstdc++
sudo apt-get install g++-4.8

sudo apt-get install boost1.55

if [[ $PDAL_CMAKE_GENERATOR == "Ninja" ]]
then
    # Need newer cmake for Ninja generator
    wget http://www.cmake.org/files/v2.8/cmake-2.8.12.2.tar.gz
    tar -xzf cmake-2.8.12.2.tar.gz
    cd cmake-2.8.12.2
    ./bootstrap
    make
    sudo make install
    cd ..

    git clone https://github.com/martine/ninja.git
    cd ninja
    git checkout release
    ./bootstrap.py
    sudo ln -s "$PWD/ninja" /usr/local/bin/ninja
    cd ..
else
    sudo apt-get install cmake
fi


# GDAL is now always required
sudo apt-get install \
    libgdal1h \
    libgdal-dev


if [[ $PDAL_OPTIONAL_COMPONENTS == "all" ]]
then
    sudo apt-get install \
        libhdf5-serial-dev \
        libproj-dev \
        libgeos++-dev \
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

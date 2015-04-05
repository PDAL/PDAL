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
sudo add-apt-repository ppa:kalakris/cmake -y
sudo add-apt-repository ppa:pdal/travis -y
sudo apt-get update -qq

# Install g++-4.8 (even if we're building clang) for updated libstdc++
sudo apt-get install g++-4.8

sudo apt-get install boost1.55
sudo apt-get install cmake

if [[ $PDAL_CMAKE_GENERATOR == "Ninja" ]]
then
    sudo apt-get install ninja-build
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
        libtiff4-dev \
        libgeotiff-dev \
        postgresql-server-dev-9.1 \
        pgpointcloud
fi

gcc --version
clang --version

#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh

sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192
sudo mv /etc/apt/sources.list.d/pgdg-source.list* /tmp
sudo apt-get update -qq

sudo apt-get install \
    software-properties-common \
    python-software-properties \
    libeigen3-dev
sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo add-apt-repository ppa:boost-latest/ppa -y
sudo add-apt-repository ppa:kalakris/cmake -y
sudo add-apt-repository ppa:pdal/travis -y
sudo apt-get update -qq

sudo apt-get -qq remove postgis

# From main
if [[ $PDAL_CMAKE_GENERATOR == "Ninja" ]]
then
    sudo apt-get install ninja-build
fi

# From ppa:ubuntu-toolchain-r/test
# Install g++-4.8 (even if we're building clang) for updated libstdc++
sudo apt-get install g++-4.8

# From ppa:boost-latest/ppa
sudo apt-get install boost1.55

# From ppa:kalakris/cmake
sudo apt-get install cmake

# From ppa:ubuntugis/ubuntugis-unstable
sudo apt-get install \
    libgdal1h \
    libgdal-dev

if [[ $PDAL_OPTIONAL_COMPONENTS == "all" ]]
then
    # From main
    sudo apt-get install \
        libflann-dev \
        libhdf5-serial-dev \
        libtiff4-dev \
        postgresql-server-dev-9.1 \
        python-numpy

    # From ppa:ubuntugis/ppa
    sudo apt-get install \
        libgeotiff-dev \
        libxml2-dev

    # From ppa:ubuntugis/ubuntugis-unstable
    sudo apt-get install \
        libgeos++-dev \
        libproj-dev

    # From ppa:pdal/travis
    sudo apt-get install \
        hexboundary \
        laz-perf \
        pgpointcloud \
        points2grid
fi

gcc --version
clang --version

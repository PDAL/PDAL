#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba update -n base -c defaults conda
mamba install -c conda-forge abseil-cpp re2 pkg-config cmake ninja compilers -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    mamba config --set channel_priority strict
    mamba install --yes draco
    mamba install --yes --quiet gdal=3.5.3 python=3.11 abseil-cpp  -y
    mamba install --yes --quiet pdal  --only-deps -y

else

    mamba install pdal --only-deps -y

fi

gdal-config --version


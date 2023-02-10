#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba update -n base -c defaults conda
mamba install -c conda-forge pkg-config cmake ninja pkgconfig -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    mamba config --set channel_priority strict
    mamba install -c conda-forge draco -y
    mamba install --yes --quiet gdal=3.5.3  python=3.11 -y
    mamba install --yes --quiet pdal  --only-deps -y

else

    mamba install pdal --only-deps -y

fi

mamba install --yes ceres-solver mkl

gdalinfo --version


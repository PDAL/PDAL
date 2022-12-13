#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda update -n base -c defaults conda
mamba install -c conda-forge abseil-cpp pkg-config cmake ninja compilers -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes draco
    conda install --yes --quiet gdal=3.5.3 python=3.11 abseil-cpp  -y
    conda install --yes --quiet pdal  --only-deps -y

else

    conda install pdal --only-deps -y

fi

gdal-config --version


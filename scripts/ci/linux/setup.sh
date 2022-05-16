#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda update -n base -c defaults conda
conda install -c conda-forge pkg-config cmake ninja pkgconfig "libstdcxx-ng>=12.1.0" -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes draco
    conda install --yes --quiet gdal=3.4.1=py310h8172e47_6 python=3.10 abseil-cpp  -y
    conda install --yes --quiet pdal  --only-deps -y

else

    conda install pdal --only-deps -y
    conda install --yes --quiet gdal=3.4.2 -y

fi

gdal-config --version


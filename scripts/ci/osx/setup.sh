#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install -c conda-forge pkg-config cmake ninja pkgconfig -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    mamba install --yes draco -y
    mamba install --yes --quiet gdal=3.4.1=py310h2eb646b_6 python=3.10
    mamba install --yes --quiet pdal  --only-deps -y

else
    mamba install --yes --quiet
    mamba install pdal  --only-deps -y

fi

gdal-config --version


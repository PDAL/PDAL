#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes libcxx=14

if [ "$BUILD_TYPE" == "fixed" ]; then
    mamba install --yes --quiet gdal=3.5.3  python=3.11
fi

gdal-config --version


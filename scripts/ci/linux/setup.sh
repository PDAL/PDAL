#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

if [ "$BUILD_TYPE" == "fixed" ]; then
    mamba install --yes --quiet gdal=3.7.1 python=3.11 abseil-cpp  -y
fi

gdal-config --version


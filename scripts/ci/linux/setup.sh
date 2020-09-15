#!/bin/bash

mkdir build
conda update -n base -c defaults conda

if [ "$BUILD_TYPE" == "fixed" ]; then
    echo "Configuring build type '$BUILD_TYPE'"

    conda install pdal --only-deps -y
    conda install cmake ninja compilers -y
else
    conda install --yes gdal=3.0.2=py37hbb6b9fb_8 python=3.7 abseil-cpp conda-build ninja
    echo "Configuring build type '$BUILD_TYPE'"
fi


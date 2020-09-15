#!/bin/bash

mkdir build

#conda config --set channel_priority strict
conda update -n base -c defaults conda
conda install cmake ninja conda-build compilers -y

echo "Configuring build type '$BUILD_TYPE'"

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes --quiet -c conda-forge gdal=3.0.2=py37hbb6b9fb_8 python=3.7 abseil-cpp  -y
    conda install --yes --quiet -c conda-forge pdal  --only-deps -y

else

    conda install pdal --only-deps -y

fi

gdal-config --version


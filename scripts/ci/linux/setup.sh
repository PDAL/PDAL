#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda update -n base -c defaults conda
conda install cmake ninja compilers -y

# can remove this line once Draco is added to the
# PDAL recipe
conda install -c conda-forge draco -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes --quiet gdal=3.0.2=py37hbb6b9fb_8 python=3.7 abseil-cpp  -y
    conda install --yes --quiet pdal  --only-deps -y

else

    conda install pdal --only-deps -y

fi

gdal-config --version


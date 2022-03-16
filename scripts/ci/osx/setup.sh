#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda install -c conda-forge python=3.9 pkg-config cmake ninja pkgconfig -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes draco -y
    conda install --yes --quiet gdal=3.2.2=py38h140b4a6_2 python=3.8  -y
    conda install --yes --quiet pdal  --only-deps -y

else

    conda install pdal --only-deps -y

fi

gdal-config --version


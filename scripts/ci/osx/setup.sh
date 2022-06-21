#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda install -c conda-forge pkg-config cmake ninja pkgconfig -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes draco -y
    conda install --yes --quiet gdal=3.4.1=py310h2eb646b_6
    conda install --yes --quiet pdal  --only-deps -y

else
    conda install --yes --quiet gdal=3.4.1=py310h2eb646b_6
    conda install pdal  --only-deps -y

fi

gdal-config --version


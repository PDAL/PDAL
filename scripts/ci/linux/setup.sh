#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda update -n base -c defaults conda
conda install -c conda-forge pkg-config cmake ninja -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes draco
    conda install --yes --quiet gdal=3.4.1=py310h8172e47_6 python=3.10 abseil-cpp  -y
    conda install --yes --quiet pdal  --only-deps -y

else

    conda install --yes --quiet gdal=3.4.2=py310hce6f0df_7 -y
    conda install --yes --quiet tiledb=2.8.3=h1e4a385_0 -y
    conda install pdal --only-deps -y

fi

gdal-config --version


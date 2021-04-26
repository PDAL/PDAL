#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

conda update -n base -c defaults conda
conda install -c conda-forge pkgconfig cmake ninja compilers -y

# can remove this after CF PDAL recipe adds Draco
conda install -c conda-forge draco -y

if [ "$BUILD_TYPE" == "fixed" ]; then

    conda config --set channel_priority strict
    conda install --yes --quiet pdal  --only-deps -y
    conda install --yes --quiet gdal=3.0.4=py38h3ba59e7_9 python=3.8 -y

else

    conda install pdal --only-deps -y

fi

gdalinfo --version


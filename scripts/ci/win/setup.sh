#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes mkl blas
mamba install -c gdal-master -c conda-forge gdal-master::gdal

gdalinfo --version


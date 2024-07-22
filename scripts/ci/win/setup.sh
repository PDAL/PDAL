#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes mkl blas

gdalinfo --version


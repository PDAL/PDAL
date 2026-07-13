#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes mkl blas curl=8.21.0=h8206538_1

gdalinfo --version


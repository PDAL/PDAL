#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes compilers ninja cmake pkg-config

gdalinfo --version


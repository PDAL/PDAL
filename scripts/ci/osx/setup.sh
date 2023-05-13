#!/bin/bash

echo "Configuring build type '$BUILD_TYPE'"
mkdir build

mamba install --yes libcxx=14

# nitro now broken due to OSX update so we need latest version
if [ "$BUILD_TYPE" == "fixed" ]; then
    mamba install --yes --quiet gdal=3.5.3  python=3.11 nitro=2.7.dev7
fi

gdal-config --version


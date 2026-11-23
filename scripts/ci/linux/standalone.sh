#!/bin/bash

CMAKE_OPTIONS="-Werror=author -Wno-error=non-target-directive --log-level=VERBOSE \
    -DCMAKE_INSTALL_BINDIR=bin -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_INCLUDEDIR=include"

declare -a arr=("i3s" "pgpointcloud" "draco" "nitf" "tiledb" "trajectory" "hdf" "icebridge" "spz")

BASE=`pwd`

for i in "${arr[@]}"
do
    cd $BASE
    cd ./plugins/$i && \
    rm -rf build ; \
    mkdir build && cd build && \
    cmake $CMAKE_OPTIONS .. \
        -DSTANDALONE=ON \
        -DPDAL_DIR="$CONDA_PREFIX/lib/cmake/PDAL" \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Debug -G Ninja && \
    ninja && \
    ninja install

done

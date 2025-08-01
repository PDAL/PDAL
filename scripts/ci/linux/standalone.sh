#!/bin/bash

declare -a arr=("i3s" "pgpointcloud" "draco" "nitf" "tiledb" "trajectory" "hdf" "icebridge" "spz")

BASE=`pwd`

for i in "${arr[@]}"
do
    cd $BASE
    cd ./plugins/$i && \
    rm -rf build ; \
    mkdir build && cd build && \
    cmake .. \
        -DSTANDALONE=ON \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Debug -G Ninja && \
    ninja && \
    ninja install

done

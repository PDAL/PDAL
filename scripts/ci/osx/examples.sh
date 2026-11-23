#!/bin/bash

CMAKE_OPTIONS="-Werror=author -Wno-error=non-target-directive --log-level=VERBOSE"

export BASE=`pwd`
for EXAMPLE in writing writing-filter writing-kernel \
    writing-reader writing-writer
do
    cd $BASE/examples/$EXAMPLE
    rm -rf _build
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake $CMAKE_OPTIONS -G "Ninja" .. \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DPDAL_DIR="$CONDA_PREFIX/lib/cmake/PDAL" && ninja
done

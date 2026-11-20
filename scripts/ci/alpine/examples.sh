#!/bin/bash

CMAKE_OPTIONS="-Werror=author -Wno-error=non-target-directive --log-level=VERBOSE"

export BASE=`pwd`
for EXAMPLE in writing writing-filter writing-kernel \
    writing-reader writing-writer
do
    cd $BASE/examples/$EXAMPLE
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake $CMAKE_OPTIONS -G "Ninja" .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DPDAL_DIR=/usr/local/lib/cmake/PDAL && ninja
done

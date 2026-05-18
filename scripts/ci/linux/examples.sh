#!/bin/bash

export BASE=`pwd`
for EXAMPLE in writing writing-filter writing-kernel \
    writing-reader writing-writer
do
    cd $BASE/examples/$EXAMPLE
    rm -rf _build
    rm -rf build
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake -G "Ninja" .. -DPDAL_DIR=$CONDA_PREFIX/lib/cmake/PDAL && cmake --build . -v
done

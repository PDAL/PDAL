#!/bin/bash

export BASE=`pwd`
export CC=cl.exe
export CXX=cl.exe
for EXAMPLE in writing writing-filter writing-kernel \
    writing-reader writing-writer
do
    cd $BASE/examples/$EXAMPLE
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake -G "Ninja" .. -DPDAL_DIR=$CONDA_PREFIX/lib/cmake/PDAL && ninja
done

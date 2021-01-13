#!/bin/bash

export BASE=`pwd`
for EXAMPLE in writing writing-filter writing-kernel \
    writing-reader writing-writer
do
    cd $BASE/examples/$EXAMPLE
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake -G "Ninja" .. && ninja
done

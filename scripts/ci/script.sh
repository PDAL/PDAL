#!/bin/bash
# Builds and tests PDAL
source ./scripts/ci/common.sh
mkdir -p _build || return 1
cd _build || return 1

cmake \
    -DWITH_GDAL=ON \
    -DWITH_GEOTIFF=ON \
    -DWITH_LIBXML2=ON \
    .. \
    || return 1

    make -j ${NUMTHREADS} || return 1

ctest -V --output-on-failure . || return 1

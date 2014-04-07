#!/bin/bash -e
# Builds and tests PDAL
source ./scripts/ci/common.sh
mkdir -p _build || exit 1
cd _build || exit 1

cmake \
    -DWITH_GDAL=ON \
    -DWITH_GEOTIFF=ON \
    -DWITH_LIBXML2=ON \
    -DWITH_PGPOINTCLOUD=ON \
    -DPDAL_EMBED_BOOST=OFF \
    ..

make -j ${NUMTHREADS}

ctest -V --output-on-failure .

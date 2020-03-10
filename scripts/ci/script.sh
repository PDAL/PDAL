#!/bin/ash -e
# Builds and tests PDAL

gcc --version
g++ --version

cd /pdal

mkdir -p _build || exit 1
cd _build || exit 1

cmake .. \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PLUGIN_CPD=ON \
    -DBUILD_PLUGIN_GREYHOUND=ON \
    -DBUILD_PLUGIN_I3S=ON \
    -DBUILD_PLUGIN_NITF=ON \
    -DBUILD_PLUGIN_ICEBRIDGE=ON \
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
    -DBUILD_PLUGIN_E57=ON \
    -DBUILD_PGPOINTCLOUD_TESTS=OFF \
    -DBUILD_PLUGIN_SQLITE=ON \
    -DWITH_LASZIP=ON \
    -DWITH_LAZPERF=ON \
    -DWITH_TESTS=ON

ninja -v
ctest -V
ninja install
cd /

for EXAMPLE in writing writing-filter writing-kernel writing-reader writing-writer
do
    cd /pdal/examples/$EXAMPLE
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake -G "Ninja" .. && \
    ninja
done

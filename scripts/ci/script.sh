#!/bin/sh -e
# Builds and tests PDAL

echo "http://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories
echo "@edgecommunity http://dl-cdn.alpinelinux.org/alpine/edge/community" >> /etc/apk/repositories; \
apk update
apk add \
    cmake \
    alpine-sdk \
    eigen-dev@edgecommunity \
    hexer \
    hexer-dev \
    nitro \
    nitro-dev \
    gdal \
    gdal-dev \
    geos \
    geos-dev \
    laz-perf \
    laz-perf-dev \
    libexecinfo \
    libexecinfo-dev \
    libgeotiff \
    libgeotiff-dev \
    libxml2 \
    libxml2-dev \
    python \
    python-dev \
    py-numpy \
    py-numpy-dev \
    cython \
    cython-dev \
    py-pip \
    jsoncpp \
    jsoncpp-dev \
    hdf5 \
    hdf5-dev \
    proj4 \
    proj4-dev \
    cpd \
    cpd-dev \
    fgt \
    fgt-dev \
    sqlite \
    sqlite-dev \
    postgresql-dev \
    libcurl \
    curl-dev \
    linux-headers \
    laszip \
    laszip-dev \
    libspatialite \
    libspatialite-dev \
    xz-dev \
    xz-libs \
    zstd \
    zstd-dev \

gcc --version
g++ --version

cd /pdal

mkdir -p _build || exit 1
cd _build || exit 1

cmake .. \
    -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc \
    -DCMAKE_CXX_COMPILER=g++ \
    -DCMAKE_MAKE_PROGRAM=make \
    -DBUILD_PLUGIN_PYTHON=ON \
    -DBUILD_PLUGIN_CPD=ON \
    -DBUILD_PLUGIN_GREYHOUND=ON \
    -DBUILD_PLUGIN_HEXBIN=ON \
    -DBUILD_PLUGIN_NITF=ON \
    -DBUILD_PLUGIN_ICEBRIDGE=ON \
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
    -DBUILD_PGPOINTCLOUD_TESTS=OFF \
    -DBUILD_PLUGIN_SQLITE=ON \
    -DWITH_LASZIP=ON \
    -DWITH_LAZPERF=ON \
    -DWITH_TESTS=ON

make -j2
LD_LIBRARY_PATH=./lib
ctest -V
make install

# Python extension testing
cd /pdal/python
pip install packaging
python setup.py build
echo "current path: " `pwd`
export PDAL_TEST_DIR=/pdal/_build/test
python setup.py test

for EXAMPLE in writing writing-filter writing-kernel writing-reader writing-writer
do
    cd /pdal/examples/$EXAMPLE
    mkdir -p _build || exit 1
    cd _build || exit 1
    cmake -G "Unix Makefiles" .. && \
    make
done

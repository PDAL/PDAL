#!/bin/sh -e
# Builds and tests PDAL

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
    -DBUILD_PLUGIN_I3S=ON \
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
pip install packaging
git clone https://github.com/PDAL/python.git pdal-python
cd pdal-python
git checkout 2.0.0
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

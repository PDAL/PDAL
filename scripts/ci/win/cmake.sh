#!/bin/bash

pwd
where cl.exe
export CC=cl.exe
export CXX=cl.exe
cmake .. --trace --trace-expand -G "Ninja" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
    -DWITH_TESTS=ON \
    -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
    -DOPENSSL_ROOT_DIR="$CONDA_PREFIX/Library" \
    -DPython3_ROOT_DIR:FILEPATH="$CONDA_PREFIX" \
    -DPython3_FIND_STRATEGY=LOCATION \
    -DUSE_EXTERNAL_GTEST=ON \
    -DCMAKE_CXX_FLAGS=" /D_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR" \
    -DENABLE_CTEST=OFF \
    -DWITH_LZMA=OFF \
    -DWITH_ZLIB=ON \
    -Dgtest_force_shared_crt=ON \
    -DBUILD_PGPOINTCLOUD_TESTS=OFF
cat build.ninja

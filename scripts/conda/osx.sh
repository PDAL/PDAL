
BUILDDIR=conda-build

#CONFIG="Unix Makefiles"
CONFIG="Ninja"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi


rm -rf $BUILDDIR
mkdir -p $BUILDDIR
cd $BUILDDIR
CC=/usr/bin/cc CXX=/usr/bin/c++ cmake   -G "$CONFIG"  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DPython3_ROOT_DIR:FILEPATH="$CONDA_PREFIX" \
        -DCMAKE_FIND_FRAMEWORK="NEVER" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DWITH_LAZPERF=ON \
        -DWITH_ZSTD=ON \
        -DWITH_LASZIP=ON \
        ..


# if ! [ -z "Unix Makefiles" ]; then
#     make -j 4
# fi

if ! [ -z "Ninja" ]; then
    ninja
fi

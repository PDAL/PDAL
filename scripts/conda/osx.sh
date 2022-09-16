
BUILDDIR=conda-build

CONFIG="Debug"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi


rm -rf $BUILDDIR
mkdir -p $BUILDDIR
cd $BUILDDIR
CFLAGS= CXXFLAGS="-Werror=strict-aliasing" CC=/usr/bin/cc CXX=/usr/bin/c++ cmake   -G "Ninja"  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DCMAKE_FIND_FRAMEWORK="NEVER" \
        -DCMAKE_BUILD_TYPE=$CONFIG \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_HDF=ON \
        -DBUILD_PLUGIN_DRACO=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DWITH_ZSTD=ON \
        ..


# if ! [ -z "Unix Makefiles" ]; then
#     make -j 4
# fi

if ! [ -z "Ninja" ]; then
    ninja
fi

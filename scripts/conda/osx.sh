
BUILDDIR=conda-build

CONFIG="Debug"
INSTALL_PREFIX="$CONDA_PREFIX"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

if ! [ -z "$2" ]; then
    INSTALL_PREFIX="$2"
fi


rm -rf $BUILDDIR
mkdir -p $BUILDDIR
cd $BUILDDIR


#SANITIZE="-fsanitize=address,alignment,unreachable,vla-bound,vptr"
#SANITIZE="-fsanitize=address,alignment,undefined"
SANITIZE=""

#CC=/usr/bin/cc
#CXX=/usr/bin/c++

cmake   -G "Ninja"  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DCMAKE_FIND_FRAMEWORK="NEVER" \
        -DCMAKE_BUILD_TYPE=$CONFIG \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DCMAKE_INSTALL_BINDIR="$INSTALL_PREFIX/bin" \
        -DCMAKE_INSTALL_LIBEXECDIR="$INSTALL_PREFIX/libexec" \
        -DCMAKE_INSTALL_LOCALEDIR="$INSTALL_PREFIX/locale" \
        -DCMAKE_INSTALL_LIBDIR="$INSTALL_PREFIX/lib" \
        -DCMAKE_INSTALL_DOCDIR="$INSTALL_PREFIX/man" \
        -DCMAKE_INSTALL_INCLUDEDIR="$INSTALL_PREFIX/include" \
        -DCMAKE_INSTALL_NAME_DIR="$INSTALL_PREFIX/lib" \
        -DCMAKE_INSTALL_SBINDIR="$INSTALL_PREFIX/sbin" \
        -DCMAKE_INSTALL_OLDINCLUDEDIR="$INSTALL_PREFIX/include" \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DCMAKE_EXE_LINKER_FLAGS="$SANITIZE" \
        -DCMAKE_CXX_FLAGS="$SANITIZE" \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_HDF=ON \
        -DBUILD_PLUGIN_ARROW=ON \
        -DBUILD_PLUGIN_DRACO=ON \
        -DBUILD_PLUGIN_E57=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_I3S_TESTS=ON \
        -DBUILD_PLUGIN_TILEDB=OFF \
        -DWITH_ZSTD=ON \
        -DWITH_LZMA=ON \
        -DWITH_TESTS=ON \
        ..


# if ! [ -z "Unix Makefiles" ]; then
#     make -j 4
# fi

if ! [ -z "Ninja" ]; then
    ninja
fi

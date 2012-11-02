USR_LOCAL="/usr/local"
USR="/usr"
TIFF_HOME=$USR_LOCAL
LASZIP_HOME=$USR_LOCAL
LIBXML2_HOME=$USR
GEOTIFF_HOME=$USR_LOCAL
SO_EXT=dylib
EMBED=ON
CC=/Users/hobu/bin/clang
CXX=/Users/hobu/bin/clang++

CONFIG="Unix Makefiles"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

cmake   -G "$CONFIG"  \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/Users/hobu \
        -DPDAL_EMBED_BOOST=${EMBED} \
        -DWITH_GDAL=ON \
        -DWITH_ICONV=ON \
        -DWITH_ORACLE=OFF \
        -DWITH_GEOTIFF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_LIBXML2=ON \
        -DWITH_PYTHON=ON \
        -DWITH_FLANN=ON \
        -DWITH_SOCI=ON \
        -DGEOTIFF_INCLUDE_DIR=${GEOTIFF_HOME}/include/ \
        -DGEOTIFF_LIBRARY=${GEOTIFF_HOME}/lib/libgeotiff.${SO_EXT} \
        -DICONV_INCLUDE_DIR=/usr/include \
        -DLASZIP_INCLUDE_DIR=${LASZIP_HOME}/include \
        -DLASZIP_LIBRARY=${LASZIP_HOME}/lib/liblaszip.${SO_EXT} \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DTIFF_INCLUDE_DIR=/${TIFF_HOME}/include \
        -DTIFF_LIBRARY=${TIFF_HOME}/lib/libtiff.${SO_EXT}

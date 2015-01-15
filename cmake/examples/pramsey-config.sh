
# Where do libs/bins go on 'make install'?
INSTALL_PREFIX=${HOME}/pdal

# Where are our dependencies installed?
TIFF_HOME=/opt/local
LASZIP_HOME=/usr/local
LIBXML2_HOME=/opt/local
GEOTIFF_HOME=/opt/local
P2G_HOME=/usr/local
PGSQL_HOME=/usr/pgsql-9.2

# What to build with?
CC=/usr/bin/gcc
CXX=/usr/bin/g++
CPPCLAGS=-I${PGSQL_HOME}/include
SO_EXT=dylib

CONFIG="Unix Makefiles"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

cmake   -G "$CONFIG" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} \
        -DWITH_GDAL=ON \
        -DWITH_ORACLE=OFF \
        -DWITH_GEOTIFF=ON \
        -DWITH_LASZIP=OFF \
        -DWITH_LIBXML2=ON \
        -DWITH_PYTHON=ON \
        -DWITH_P2G=OFF \
        -DWITH_SOCI=OFF \
        -DWITH_PGPOINTCLOUD=ON \
        -DGEOTIFF_INCLUDE_DIR=${GEOTIFF_HOME}/include \
        -DGEOTIFF_LIBRARY=${GEOTIFF_HOME}/lib/libgeotiff.${SO_EXT} \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DTIFF_INCLUDE_DIR=/${TIFF_HOME}/include \
        -DTIFF_LIBRARY=${TIFF_HOME}/lib/libtiff.${SO_EXT} \
	-DPOSTGRESQL_INCLUDE_DIR=${PGSQL_HOME}/include \
	-DPOSTGRESQL_LIBRARIES=${PGSQL_HOME}/lib/libpq.${SO_EXT} \
	../PDAL



#	-DLASZIP_INCLUDE_DIR=${LASZIP_HOME}/include \
#	-DLASZIP_LIBRARY=${LASZIP_HOME}/lib/liblaszip.${SO_EXT} \

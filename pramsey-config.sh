USR_LOCAL="/usr/local"
USR="/usr"
OPT="/opt/local"
TIFF_HOME=$USR_LOCAL
LASZIP_HOME=$USR_LOCAL
LIBXML2_HOME=$OPT
GEOTIFF_HOME=$USR_LOCAL
P2G_HOME=$USR_LOCAL
SO_EXT=dylib
EMBED=ON
CC=/usr/bin/gcc
CXX=/usr/bin/g++
CPPCLAGS=-I/usr/pgsql-9.2/include

ORACLE_HOME=$HOME/oracle
export ORACLE_HOME
CONFIG="Unix Makefiles"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

cmake   -G "$CONFIG"  \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_INSTALL_PREFIX=${HOME}/pdal \
        -DPDAL_EMBED_BOOST=${EMBED} \
        -DWITH_GDAL=ON \
        -DWITH_ICONV=ON \
        -DWITH_ORACLE=OFF \
        -DWITH_GEOTIFF=ON \
        -DWITH_LASZIP=OFF \
        -DWITH_LIBXML2=ON \
        -DWITH_PYTHON=ON \
        -DWITH_FLANN=OFF \
        -DWITH_P2G=OFF \
        -DWITH_SOCI=OFF \
        -DWITH_PGPOINTCLOUD=ON \
        -DGEOTIFF_INCLUDE_DIR=${GEOTIFF_HOME}/include \
        -DGEOTIFF_LIBRARY=${GEOTIFF_HOME}/lib/libgeotiff.${SO_EXT} \
        -DICONV_INCLUDE_DIR=/usr/include \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DTIFF_INCLUDE_DIR=/${TIFF_HOME}/include \
        -DTIFF_LIBRARY=${TIFF_HOME}/lib/libtiff.${SO_EXT} \
	-DPOSTGRESQL_INCLUDE_DIR=${USR_LOCAL}/pgsql/9.2/include \
	-DPOSTGRESQL_LIBRARIES=${USR_LOCAL}/pgsql/9.2/lib/libpq.${SO_EXT} \
	../PDAL


# -DLASZIP_INCLUDE_DIR=${LASZIP_HOME}/include \
# -DLASZIP_LIBRARY=${LASZIP_HOME}/lib/liblaszip.${SO_EXT} \
# -DUSE_PDAL_PLUGIN_SOCI=ON \
# -DUSE_PDAL_PLUGIN_PCD=ON \
# -DUSE_PDAL_PLUGIN_OCI=ON \
# -DUSE_PDAL_PLUGIN_TEXT=ON \

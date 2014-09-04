USR_LOCAL="/usr/local"
USR="/usr"
TIFF_HOME=$USR_LOCAL
LASZIP_HOME=$USR_LOCAL
LIBXML2_HOME=$USR
GEOTIFF_HOME=$USR_LOCAL
P2G_HOME=$USR_LOCAL
HEXER_HOME="/usr/local"
SQLITE_HOME="/usr/local/opt/sqlite"
SO_EXT=dylib
# CC=/usr/local/bin/gcc-4.8
# CXX=/usr/local/bin/g++-4.8

ORACLE_HOME=$HOME/oracle
export ORACLE_HOME
# CONFIG="Unix Makefiles"
CONFIG="Ninja"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

CC=$CC CXX=$CXX cmake   -G "$CONFIG"  \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/Users/hobu \
        -DWITH_ORACLE=ON \
        -DWITH_GEOTIFF=ON \
        -DWITH_SQLITE=ON \
        -DWITH_P2G=ON \
        -DWITH_HEXER=ON \
        -DWITH_NITRO=ON \
        -DWITH_MRSID=OFF \
        -DWITH_HDF5=ON \
        -DWITH_PCL=ON \
        -DWITH_PGPOINTCLOUD=ON \
        -DPCL_DIR=/Users/hobu/dev/git/pcl/installed/share/pcl-1.7/ \
        -DMRSID_INCLUDE_DIR=/Users/hobu/dev/release/mrsid/Lidar_DSDK/include \
        -DMRSID_LIBRARY=/Users/hobu/dev/release/mrsid/Lidar_DSDK/lib/liblti_lidar_dsdk.dylib \
        -DHEXER_INCLUDE_DIR=${HEXER_HOME}/include \
        -DHEXER_LIBRARY=${HEXER_HOME}/lib/libhexer.${SO_EXT} \
        -DGEOTIFF_INCLUDE_DIR=${GEOTIFF_HOME}/include/ \
        -DGEOTIFF_LIBRARY=${GEOTIFF_HOME}/lib/libgeotiff.${SO_EXT} \
        -DICONV_INCLUDE_DIR=/usr/include \
        -DP2G_INCLUDE_DIR=${P2G_HOME}/include \
        -DP2G_LIBRARY=${P2G_HOME}/lib/libpts2grd.${SO_EXT} \
        -DLASZIP_INCLUDE_DIR=${LASZIP_HOME}/include \
        -DLASZIP_LIBRARY=${LASZIP_HOME}/lib/liblaszip.${SO_EXT} \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DSQLITE3_INCLUDE_DIR=${SQLITE_HOME}/include \
        -DSQLITE3_LIBRARY=${SQLITE_HOME}/lib/libsqlite3.${SO_EXT}

    # -DUSE_PDAL_PLUGIN_SOCI=ON \
    # -DUSE_PDAL_PLUGIN_PCD=ON \
    # -DUSE_PDAL_PLUGIN_OCI=ON \
    # -DUSE_PDAL_PLUGIN_TEXT=ON \

        # -DPYTHON_EXECUTABLE=/usr/local/bin/python3 \
        # -DPYTHON_LIBRARY=/usr/local/Cellar/python3/3.3.3/Frameworks/Python.framework/Versions/3.3/lib/libpython3.3.dylib \
        # -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python3/3.3.3/Frameworks/Python.framework/Versions/3.3/include/python3.3m/ \

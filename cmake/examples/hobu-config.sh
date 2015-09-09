USR_LOCAL="/usr/local"
USR="/usr"
TIFF_HOME=$USR_LOCAL
LASZIP_HOME=$USR_LOCAL
LIBXML2_HOME=/usr/local/Cellar/libxml2/2.9.2/
WEBSOCKETPP_HOME=/Users/hobu/dev/git/websocketpp
GEOTIFF_HOME=$USR_LOCAL
P2G_HOME=$USR_LOCAL
HEXER_HOME="/usr/local"
SQLITE_HOME="/usr/local/opt/sqlite"
SO_EXT=dylib
# CC=/usr/local/bin/gcc-4.8
# CXX=/usr/local/bin/g++-4.8

ORACLE_HOME=$HOME/oracle
LAZPERF_HOME=$USR_LOCAL
export ORACLE_HOME
CONFIG="Unix Makefiles"
#CONFIG="Ninja"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

CC=$CC CXX=$CXX cmake   -G "$CONFIG"  \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/Users/hobu \
        -DBUILD_PLUGIN_OCI=ON \
        -DWITH_GEOTIFF=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_OCI_TESTS=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_P2G=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_MRSID=ON \
        -DBUILD_PLUGIN_CPD=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_PCL=ON \
        -DBUILD_PLUGIN_ATTRIBUTE=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DWITH_LAZPERF=ON \
        -DMRSID_INCLUDE_DIR=/Users/hobu/dev/release/mrsid/Lidar_DSDK/include \
        -DMRSID_LIBRARY=/Users/hobu/dev/release/mrsid/Lidar_DSDK/lib/liblti_lidar_dsdk.dylib \
        -DHEXER_INCLUDE_DIR=${HEXER_HOME}/include \
        -DHEXER_LIBRARY=${HEXER_HOME}/lib/libhexer.${SO_EXT} \
        -DGEOTIFF_INCLUDE_DIR=${GEOTIFF_HOME}/include/ \
        -DGEOTIFF_LIBRARY=${GEOTIFF_HOME}/lib/libgeotiff.${SO_EXT} \
        -DP2G_INCLUDE_DIR=${P2G_HOME}/include \
        -DP2G_LIBRARY=${P2G_HOME}/lib/libpts2grd.${SO_EXT} \
        -DLASZIP_INCLUDE_DIR=${LASZIP_HOME}/include \
        -DLASZIP_LIBRARY=${LASZIP_HOME}/lib/liblaszip.${SO_EXT} \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DLazperf_DIR=${LAZPERF_HOME}/ \
        -DJSONCPP_ROOT_DIR=${USR_LOCAL}/include/jsoncpp \
        -DSQLITE3_INCLUDE_DIR=${SQLITE_HOME}/include \
        -DSQLITE3_LIBRARY=${SQLITE_HOME}/lib/libsqlite3.${SO_EXT} \
        -DPYTHON_EXECUTABLE=/usr/local/bin/python3 \
        -DPYTHON_LIBRARY=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/lib/libpython3.4.dylib \
        -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/include/python3.4m/ \
        -DOCI_CONNECTION="grid/grid@localhost/vm" \
        -DPGPOINTCLOUD_TEST_DB_HOST="localhost"


#         -DPYTHON_EXECUTABLE=/usr/local/bin/python \
#         -DPYTHON_LIBRARY=/usr/local/Cellar/python/2.7.10_2/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib \
#         -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python/2.7.10_2/Frameworks/Python.framework/Versions/2.7/include/python2.7/ \

#         -DPYTHON_EXECUTABLE=/usr/local/bin/python3 \
#         -DPYTHON_LIBRARY=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/lib/libpython3.4.dylib \
#         -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/include/python3.4m/ \

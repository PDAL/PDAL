USR_LOCAL="/usr/local"
USR="/usr"
TIFF_HOME=$USR_LOCAL
LIBXML2_HOME=/usr/local/Cellar/libxml2/2.9.9_2/
WEBSOCKETPP_HOME=/Users/hobu/dev/git/websocketpp
SQLITE_HOME="/usr/local/opt/sqlite"
SO_EXT=dylib
#CC=/usr/local/bin/gcc-5
#CXX=/usr/local/bin/g++-5

ORACLE_HOME=$HOME/oracle
LAZPERF_HOME=$USR_LOCAL
export ORACLE_HOME
export GDAL_HOME=/usr/local/opt/gdal2

CONFIG="Unix Makefiles"
CONFIG="Ninja"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi

CC=$CC CXX=$CXX cmake   -G "$CONFIG"  \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=/Users/hobu/pdal-build \
        -DBUILD_PLUGIN_OCI=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_OCI_TESTS=ON \
        -DBUILD_I3S_TESTS=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_MRSID=ON \
        -DBUILD_PLUGIN_MBIO=ON \
        -DBUILD_PLUGIN_CPD=OFF \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_I3S=ON \
        -DBUILD_PLUGIN_RDBLIB=ON \
        -DBUILD_PLUGIN_OPENSCENEGRAPH=OFF \
        -DWITH_LAZPERF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_EXAMPLES=ON \
        -Drdb_DIR=/Users/hobu/dev/release/riegl/rdblib-2.1.5-x86_64-darwin/interface/cpp \
        -DCURL_INCLUDE_DIR=/usr/local/opt/curl/include \
        -DCURL_LIBRARY=/usr/local/opt/curl/lib/libcurl.dylib \
        -DMRSID_INCLUDE_DIR=/Users/hobu/dev/release/mrsid/Lidar_DSDK/include \
        -DMRSID_LIBRARY=/Users/hobu/dev/release/mrsid/Lidar_DSDK/lib/liblti_lidar_dsdk.dylib \
        -DLIBXML2_INCLUDE_DIR=${LIBXML2_HOME}/include/libxml2 \
        -DLIBXML2_LIBRARIES=${LIBXML2_HOME}/lib/libxml2.${SO_EXT} \
        -DLazperf_DIR=${LAZPERF_HOME}/ \
        -DSQLITE3_INCLUDE_DIR=${SQLITE_HOME}/include \
        -DSQLITE3_LIBRARY=${SQLITE_HOME}/lib/libsqlite3.${SO_EXT} \
        -DPYTHON_EXECUTABLE=python \
        -DPYTHON_LIBRARY=/usr/local/Cellar/python/3.7.2_2/Frameworks/Python.framework/Versions/3.7/lib/libpython3.7.dylib \
        -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python/3.7.2_2/Frameworks/Python.framework/Versions/3.7/include/python3.7m/\
        -DOCI_CONNECTION="lidar/lidar@localhost:1521/xe.oracle.docker" \
        -DPGPOINTCLOUD_TEST_DB_HOST="localhost"
#        -DBUILD_PLUGIN_MATLAB=ON \
#         -DMATLAB_MEX_LIBRARY=/Applications/MATLAB_R2017b.app/bin/maci64/libmex.dylib \
#         -DMATLAB_MAT_LIBRARY=/Applications/MATLAB_R2017b.app/bin/maci64/libmat.dylib \
#         -DMATLAB_MX_LIBRARY=/Applications/MATLAB_R2017b.app/bin/maci64/libmx.dylib \
#         -DMATLAB_INCLUDE_DIR=/Applications/MATLAB_R2017b.app/extern/include \
#         -DMATLAB_ENG_LIBRARY=/Applications/MATLAB_R2017b.app/bin/maci64/libeng.dylib


#         -DPYTHON_EXECUTABLE=/usr/local/bin/python \
#         -DPYTHON_LIBRARY=/usr/local/Cellar/python/2.7.10_2/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib \
#         -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python/2.7.10_2/Frameworks/Python.framework/Versions/2.7/include/python2.7/ \

#         -DPYTHON_EXECUTABLE=/usr/local/bin/python3 \
#         -DPYTHON_LIBRARY=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/lib/libpython3.4.dylib \
#         -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python3/3.4.2_1/Frameworks/Python.framework/Versions/3.4/include/python3.4m/ \

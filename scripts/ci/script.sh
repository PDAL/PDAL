#!/bin/bash -e
# Builds and tests PDAL
source ./scripts/ci/common.sh

mkdir -p _build || exit 1
cd _build || exit 1

case "$PDAL_OPTIONAL_COMPONENTS" in
    all)
        OPTIONAL_COMPONENT_SWITCH=ON
        ;;
    none)
        OPTIONAL_COMPONENT_SWITCH=OFF
        ;;
    *)
        echo "Unrecognized value for PDAL_OPTIONAL_COMPONENTS=$PDAL_OPTIONAL_COMPONENTS"
        exit 1
esac

if [[ "$CXX" == "g++" ]]
then
    export CXX="g++-4.8"
fi

cmake \
    -DBUILD_PLUGIN_ATTRIBUTE=ON \
    -DBUILD_PLUGIN_GREYHOUND=OFF \
    -DBUILD_PLUGIN_HEXBIN=OFF \
    -DBUILD_PLUGIN_ICEBRIDGE=OFF \
    -DBUILD_PLUGIN_MRSID=OFF \
    -DBUILD_PLUGIN_NITF=OFF \
    -DBUILD_PLUGIN_OCI=OFF \
    -DBUILD_PLUGIN_P2G=OFF \
    -DBUILD_PLUGIN_PCL=OFF \
    -DBUILD_PLUGIN_PGPOINTCLOUD=OFF\
    -DBUILD_PLUGIN_SQLITE=OFF \
    -DBUILD_PLUGIN_RIVLIB=OFF \
    -DENABLE_CTEST=OFF \
    -DWITH_APPS=ON \
    -DWITH_COMPRESSION=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_GEOS=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_GEOTIFF=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_ICONV=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LASZIP=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LIBXML2=OFF \
    -DWITH_PKGCONFIG=ON \
    -DWITH_PYTHON=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_TESTS=ON \
    -G "$PDAL_CMAKE_GENERATOR" \
    ..

if [[ $PDAL_CMAKE_GENERATOR == "Unix Makefiles" ]]
then
    MAKECMD=make
else
    MAKECMD=ninja
fi

# Don't use ninja's default number of threads becuase it can
# saturate Travis's available memory.
${MAKECMD} -j ${NUMTHREADS} && \
    LD_LIBRARY_PATH=./lib ctest -V && \
    sudo ${MAKECMD} install

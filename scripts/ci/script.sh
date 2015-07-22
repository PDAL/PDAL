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
    -DBUILD_PLUGIN_ATTRIBUTE=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_CPD=OFF \
    -DBUILD_PLUGIN_GREYHOUND=OFF \
    -DBUILD_PLUGIN_HEXBIN=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_ICEBRIDGE=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_MRSID=OFF \
    -DBUILD_PLUGIN_NITF=OFF \
    -DBUILD_PLUGIN_OCI=OFF \
    -DBUILD_PLUGIN_P2G=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_PCL=OFF \
    -DBUILD_PLUGIN_PGPOINTCLOUD=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_SQLITE=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_RIVLIB=OFF \
    -DBUILD_PLUGIN_PYTHON=$OPTIONAL_COMPONENT_SWITCH \
    -DENABLE_CTEST=OFF \
    -DWITH_APPS=ON \
    -DWITH_LAZPERF=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_GEOTIFF=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LASZIP=$OPTIONAL_COMPONENT_SWITCH \
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
    LD_LIBRARY_PATH=./lib && \
    sudo PGUSER=postgres ctest -V && \
    sudo ${MAKECMD} install

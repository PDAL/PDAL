#!/bin/bash -e
# Builds and tests PDAL

clang --version
gcc --version

cd /pdal
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


cmake \
    -DBUILD_PLUGIN_CPD=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_GREYHOUND=OFF \
    -DBUILD_PLUGIN_HEXBIN=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_ICEBRIDGE=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_MRSID=OFF \
    -DBUILD_PLUGIN_NITF=OFF \
    -DBUILD_PLUGIN_OCI=OFF \
    -DBUILD_PLUGIN_OPENSCENEGRAPH=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_PCL=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_PGPOINTCLOUD=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PGPOINTCLOUD_TESTS=OFF \
    -DBUILD_PLUGIN_SQLITE=$OPTIONAL_COMPONENT_SWITCH \
    -DBUILD_PLUGIN_RIVLIB=OFF \
    -DBUILD_PLUGIN_PYTHON=$OPTIONAL_COMPONENT_SWITCH \
    -DENABLE_CTEST=OFF \
    -DWITH_APPS=ON \
    -DWITH_LAZPERF=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LASZIP=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_PDAL_JNI=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_TESTS=ON \
    -G "$PDAL_CMAKE_GENERATOR" \
    ..

cmake ..

MAKECMD=make

# Don't use ninja's default number of threads becuase it can
# saturate Travis's available memory.
NUMTHREADS=2
${MAKECMD} -j ${NUMTHREADS} && \
    LD_LIBRARY_PATH=./lib && \
    PGUSER=postgres ctest -V && \
    ${MAKECMD} install && \
    /sbin/ldconfig

if [ "${OPTIONAL_COMPONENT_SWITCH}" == "ON" ]; then
    cd /pdal/python
    pip install packaging
    python setup.py build
    echo "current path: " `pwd`
    export PDAL_TEST_DIR=/pdal/_build/test
    python setup.py test

    # JNI tests
#    cd /pdal/java; PDAL_DEPEND_ON_NATIVE=false ./sbt -Djava.library.path=/pdal/_build/lib core/test

    # Build all examples
    for EXAMPLE in writing writing-filter writing-kernel writing-reader writing-writer
    do
        cd /pdal/examples/$EXAMPLE
        mkdir -p _build || exit 1
        cd _build || exit 1
        cmake -G "$PDAL_CMAKE_GENERATOR" .. && \
        ${MAKECMD}
    done
fi

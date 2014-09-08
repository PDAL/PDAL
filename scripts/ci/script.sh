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
    -DWITH_APPS=ON \
    -DWITH_TESTS=ON \
    -DWITH_PKGCONFIG=ON \
    -DWITH_GDAL=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_GEOTIFF=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_ORACLE=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_ICONV=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LASZIP=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_LIBXML2=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_MSGPACK=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_NITRO=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_PGPOINTCLOUD=OFF\
    -DWITH_PYTHON=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_ZLIB=$OPTIONAL_COMPONENT_SWITCH \
    -DWITH_CARIS=OFF \
    -DWITH_SQLITE=OFF \
    -DENABLE_CTEST=OFF \
    -DWITH_HDF5=OFF \
    -DWITH_STUBS=ON \
    -G "$PDAL_CMAKE_GENERATOR" \
    ..

if [[ $PDAL_CMAKE_GENERATOR == "Unix Makefiles" ]]
then
    make -j ${NUMTHREADS}
else
    # Don't use ninja's default number of threads becuase it can
    # saturate Travis's available memory.
    ninja -j ${NUMTHREADS}
fi

#LD_LIBRARY_PATH=./lib ctest -V --output-on-failure .
LD_LIBRARY_PATH=./lib ./bin/pdal_test "../test/data" "--catch_system_errors=no"

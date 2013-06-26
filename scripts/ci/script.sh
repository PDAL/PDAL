#!/bin/bash
# Builds and tests PDAL
source ./scripts/ci/common.sh
mkdir -p _build
cd _build

echo "$(tmstamp) *** script::cmake-config starting $(date) ***"
cmake ..
echo "$(tmstamp) *** script::cmake-config finished $(date) ***"

echo "$(tmstamp) *** script::cmake-build make -j ${NUMTHREADS} $(date) ***"
make -j ${NUMTHREADS}
echo "$(tmstamp) *** script::cmake-build finished $(date) ***"

echo "$(tmstamp) *** script::cmake-test starting $(date) ***"
ctest -V --output-on-failure .
echo "$(tmstamp) *** script::cmake-test finished $(date) ***"

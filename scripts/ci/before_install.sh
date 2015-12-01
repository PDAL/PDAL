#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh

docker pull pdal/dependencies
docker ps -a
docker run pdal/dependencies

gcc --version
clang --version

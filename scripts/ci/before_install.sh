#!/bin/bash -e
# Installs requirements for PDAL
source ./scripts/ci/common.sh

docker pull pdal/dependencies

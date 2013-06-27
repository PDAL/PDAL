#!/bin/bash
# Installs requirements for PDAL
source ./scripts/ci/common.sh
sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192 || return 1
sudo apt-get install python-software-properties -y || return 1
sudo add-apt-repository ppa:ubuntugis/ppa -y || return 1
sudo apt-get update -qq || return 1
sudo apt-get install cmake libgdal-dev libgeotiff-dev libproj-dev libxml2-dev || return 1
gcc --version || return 1
clang --version || return 1

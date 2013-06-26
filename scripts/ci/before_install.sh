#!/bin/bash
# Installs requirements for PDAL
source ./scripts/ci/common.sh
#echo "$(tmstamp) *** before_install::apt-get starting $(date) ***"
sudo apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 16126D3A3E5C1192
sudo apt-get install python-software-properties -y
sudo add-apt-repository ppa:ubuntugis/ppa -y
sudo apt-get update -qq
sudo apt-get install cmake libgdal-dev libproj-dev libxml2-dev
#echo "$(tmstamp) *** before_install::apt-get finished $(date) ***"
gcc --version
clang --version

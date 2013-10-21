#!/bin/bash -e
# Installs SOCI library

git clone https://github.com/hobu/soci.git soci
cd soci
git checkout develop
cmake . -DCMAKE_INSTALL_PREFIX=/usr -DWITH_BOOST=ON
make
sudo make install
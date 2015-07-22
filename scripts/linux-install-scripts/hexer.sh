#!/bin/bash -e
# Installs hexer library

git clone https://github.com/hobu/hexer.git
cd hexer
cmake . -DCMAKE_INSTALL_PREFIX=/usr -DWITH_GDAL=ON
make
sudo make install
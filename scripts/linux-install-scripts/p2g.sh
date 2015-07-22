#!/bin/bash -e
# Installs points2grid library

git clone https://github.com/CRREL/points2grid.git
cd points2grid
cmake . -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
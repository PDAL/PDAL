#!/bin/bash -e
# Installs NITRO library

git clone https://github.com/hobu/nitro.git
cd nitro
cmake . -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
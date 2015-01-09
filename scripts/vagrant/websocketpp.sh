#!/bin/bash -e
# Installs websocketpp library

git clone https://github.com/zaphoyd/websocketpp.git
cd websocketpp
cmake . -DCMAKE_INSTALL_PREFIX=/usr
sudo make install

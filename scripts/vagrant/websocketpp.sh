#!/bin/bash -e
# Installs websocketpp library

git clone https://github.com/zaphoyd/websocketpp.git
cmake .
sudo make install

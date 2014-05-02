#!/bin/bash
# Clean stuff up

find . -name cmake_install.cmake -exec rm {} \;
find . -name CMakeCache.txt -exec rm {} \;
find . -name CMakeFiles -exec rm -rf {} \;

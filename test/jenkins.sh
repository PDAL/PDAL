#!/usr/bin/env sh
# Run the tests in a clean PDAL repository
# clean == git clean -fdx

cmake -G "Unix Makefiles" -C ../BuildSetup.cmake .
make
./bin/pdal_test test/data


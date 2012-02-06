#!/usr/bin/env sh
# Run the tests in a clean PDAL repository
# "clean" == git clean -fdx

cmake -G "Unix Makefiles" -C "test/BuildSetup.jenkins" .
make
./bin/pdal_test test/data
